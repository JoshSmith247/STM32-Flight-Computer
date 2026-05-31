import json
import math
import socket
import struct
import threading
import time

import config

try:
    from pymavlink import mavutil as _mavutil
    _HAVE_MAVLINK = True
except ImportError:
    _HAVE_MAVLINK = False

# ---------------------------------------------------------------------------
# Weed target sender — UDP to Pi weed_pilot.py
# ---------------------------------------------------------------------------

_target_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ---------------------------------------------------------------------------
# GCS → STM32 command sender
# ---------------------------------------------------------------------------
# Builds a minimal MAVLink v2 COMMAND_LONG frame and sends it as UDP to the
# Pi's GCS port; the Pi's _gcs_to_serial thread relays it to the STM32.

_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_cmd_seq  = 0


def _cmd_crc(data: bytes, extra: int) -> int:
    crc = 0xFFFF
    for b in list(data) + [extra]:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def send_mavlink_command(cmd: int, param1: float = 0.0, param2: float = 0.0) -> None:
    """Send MAVLink v2 COMMAND_LONG to the STM32 via the Pi serial relay."""
    global _cmd_seq
    # payload: 7×f32 params + u16 cmd + target_sys + target_comp + confirmation
    payload = struct.pack('<7fH', param1, param2, 0.0, 0.0, 0.0, 0.0, 0.0, cmd)
    payload += bytes([1, 1, 0])          # target_sys=1, target_comp=1, confirmation=0
    n      = len(payload)                # 33 bytes
    seq    = _cmd_seq & 0xFF
    _cmd_seq += 1
    header = bytes([n, 0, 0, seq, 255, 0, 76, 0, 0])   # msgid=76 COMMAND_LONG
    crc    = _cmd_crc(header + payload, 152)             # CRC_EXTRA for COMMAND_LONG
    frame  = bytes([0xFD]) + header + payload + struct.pack('<H', crc)
    try:
        _cmd_sock.sendto(frame, (config.PI_IP, config.GCS_PORT))
        print(f"→ STM32 cmd={cmd} param1={param1} param2={param2}", flush=True)
    except OSError as exc:
        print(f"send_mavlink_command: {exc}", flush=True)


def send_weed_target(wid: int, east_m: float, north_m: float) -> None:
    """Send selected weed's world position to the Pi over UDP."""
    msg = json.dumps({'wid': wid, 'east_m': round(east_m, 3),
                      'north_m': round(north_m, 3)}).encode()
    try:
        _target_sock.sendto(msg, (config.PI_IP, config.WEED_TARGET_PORT))
        print(f"Sent W{wid} → Pi: {east_m:.2f}m E, {north_m:.2f}m N", flush=True)
    except OSError as exc:
        print(f"UDP send failed: {exc}", flush=True)


# ---------------------------------------------------------------------------
# MAVLink position receiver
# ---------------------------------------------------------------------------
# Receives GLOBAL_POSITION_INT + ATTITUDE from the drone in a background thread.
# pixel_to_world() uses the latest fix to project pixel centroids to local ENU metres,
# giving each named weed a world-frame position that survives it leaving the frame.

_mav_lock  = threading.Lock()
_mav_state: dict = {
    'lat': None, 'lon': None, 'alt': 1.0, 'yaw': 0.0, 'origin': None,
    'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'last_msg_t': None,
    'roll': 0.0, 'pitch': 0.0, 'battery_pct': -1,
    'armed': False, 'mode': '',
    'voltage_mv': -1, 'current_ca': -1,
    'mah_consumed': -1, 'time_remaining_s': 0,
    'motor_pwm': [0, 0, 0, 0],
    # Decoded from HEARTBEAT custom_mode (see telemetry.rs build_heartbeat)
    'flight_mode_id': 0,   # FlightMode enum value [7:0]
    'flight_state':   0,   # FlightState enum value [15:8]
    'payload_flags':  0,   # connected payload bitmask [31:16]
}


def _mav_listener() -> None:
    if not _HAVE_MAVLINK:
        return
    try:
        conn = _mavutil.mavlink_connection(f'udpin:0.0.0.0:{config.GCS_PORT}')
    except Exception as exc:
        print(f"MAVLink listener failed: {exc}", flush=True)
        return
    print(f"MAVLink listening on UDP port {config.GCS_PORT}", flush=True)
    while True:
        msg = conn.recv_match(
            type=['GLOBAL_POSITION_INT', 'ATTITUDE', 'SYS_STATUS',
                  'HEARTBEAT', 'BATTERY_STATUS', 'SERVO_OUTPUT_RAW'],
            blocking=True, timeout=1.0)
        if msg is None:
            continue
        with _mav_lock:
            t = time.monotonic()
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = max(0.3, msg.relative_alt / 1000.0)   # mm → m, clamp
                if _mav_state['origin'] is None:
                    _mav_state['origin'] = (lat, lon)
                _mav_state.update(lat=lat, lon=lon, alt=alt,
                                  vx=msg.vx / 100.0, vy=msg.vy / 100.0,
                                  vz=msg.vz / 100.0, last_msg_t=t)
            elif msg.get_type() == 'ATTITUDE':
                _mav_state.update(roll=msg.roll, pitch=msg.pitch,
                                  yaw=msg.yaw, last_msg_t=t)
            elif msg.get_type() == 'SYS_STATUS':
                _mav_state['battery_pct'] = msg.battery_remaining
            elif msg.get_type() == 'HEARTBEAT':
                cm = msg.custom_mode
                _mav_state['armed']          = bool(msg.base_mode & 128)
                _mav_state['flight_mode_id'] = cm & 0xFF
                _mav_state['flight_state']   = (cm >> 8) & 0xFF
                _mav_state['payload_flags']  = (cm >> 16) & 0xFFFF
            elif msg.get_type() == 'BATTERY_STATUS':
                valid = [v for v in msg.voltages if v != 65535]
                _mav_state.update(
                    voltage_mv       = sum(valid) if valid else -1,
                    current_ca       = msg.current_battery,
                    mah_consumed     = msg.current_consumed,
                    time_remaining_s = msg.time_remaining,
                    battery_pct      = msg.battery_remaining,
                )
            elif msg.get_type() == 'SERVO_OUTPUT_RAW':
                _mav_state['motor_pwm'] = [
                    msg.servo1_raw, msg.servo2_raw,
                    msg.servo3_raw, msg.servo4_raw,
                ]


def pixel_to_world(px: float, py: float,
                   frame_w: int, frame_h: int) -> tuple[float, float] | None:
    """Project a pixel centroid to local ENU metres (East, North).

    Assumes nadir camera: image-right = drone-right, image-up = drone-forward.
    Returns None until the first MAVLink GLOBAL_POSITION_INT is received.
    """
    with _mav_lock:
        if _mav_state['lat'] is None:
            return None
        lat, lon, alt, yaw, orig = (
            _mav_state['lat'], _mav_state['lon'],
            _mav_state['alt'], _mav_state['yaw'], _mav_state['origin'],
        )

    # Pixel offset from principal point → drone-body metres
    fx    = (frame_w / 2) / math.tan(math.radians(config.CAM_HFOV / 2))
    scale = alt / fx
    u =  (px - frame_w / 2) * scale   # rightward  (East  when yaw = 0)
    v = -(py - frame_h / 2) * scale   # forward    (North when yaw = 0, y-axis flipped)

    # Rotate body frame into ENU by drone yaw
    east  =  math.cos(yaw) * u + math.sin(yaw) * v
    north = -math.sin(yaw) * u + math.cos(yaw) * v

    # Drone ENU position relative to first fix
    R = 6_371_000.0
    drone_e = math.radians(lon - orig[1]) * R * math.cos(math.radians(orig[0]))
    drone_n = math.radians(lat - orig[0]) * R

    return drone_e + east, drone_n + north


# ---------------------------------------------------------------------------
# MAVLink targeting stub (commented out — ExG-only mode)
# ---------------------------------------------------------------------------

# def send_target(sock: socket.socket, x_norm: float, y_norm: float) -> None:
#     print(f"Target offset: x={x_norm:+.2f} y={y_norm:+.2f}", flush=True)
#     # TODO: sock.sendto(mavlink_frame, (PI_IP, GCS_PORT))
