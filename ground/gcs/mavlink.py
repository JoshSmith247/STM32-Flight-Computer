import datetime
import json
import math
import os
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
    # GPS_RAW_INT fix quality (0=no GPS, 1=no fix, 2=2D, 3=3D, 4+=DGPS/RTK)
    'gps_fix_type':   0,
    # VFR_HUD collective throttle percentage (0–100)
    'throttle_pct':   0,
    # SYS_STATUS onboard_control_sensors_health bitmask (0 = not yet received)
    'sensors_health': 0,
    # Last COMMAND_ACK received — (cmd_id, result_code); consumed by draw_stats_panel
    'last_ack': None,
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
                  'HEARTBEAT', 'BATTERY_STATUS', 'SERVO_OUTPUT_RAW', 'COMMAND_ACK',
                  'GPS_RAW_INT', 'VFR_HUD'],
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
                if _mav_state['battery_pct'] < 0:   # BATTERY_STATUS is authoritative once received
                    _mav_state['battery_pct'] = msg.battery_remaining
                _mav_state['sensors_health'] = msg.onboard_control_sensors_health
            elif msg.get_type() == 'GPS_RAW_INT':
                _mav_state['gps_fix_type'] = msg.fix_type
            elif msg.get_type() == 'VFR_HUD':
                _mav_state['throttle_pct'] = msg.throttle
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
            elif msg.get_type() == 'COMMAND_ACK':
                _RESULT = {0: 'ACCEPTED', 1: 'DENIED', 2: 'UNSUPPORTED',
                           3: 'FAILED', 4: 'IN_PROGRESS'}
                _mav_state['last_ack'] = (msg.command, msg.result)
                if msg.result != 0:
                    print(f"COMMAND_ACK cmd={msg.command} "
                          f"result={_RESULT.get(msg.result, msg.result)}", flush=True)


def pixel_to_world(px: float, py: float,
                   frame_w: int, frame_h: int) -> tuple[float, float] | None:
    """Project a pixel centroid to local ENU metres (East, North).

    Uses a 3-D ray → ground-plane intersection that accounts for drone tilt
    (roll and pitch), not just heading. When the drone tilts to accelerate, the
    camera tilts with it; a pure yaw rotation of pre-scaled offsets would produce
    incorrect weed coordinates.

    Camera assumed nadir-pointing: image +X = body +Y (right), image +Y = body +X (forward).
    Returns None until the first MAVLink GLOBAL_POSITION_INT is received.
    """
    with _mav_lock:
        if _mav_state['lat'] is None:
            return None
        lat, lon, alt, yaw, orig, roll, pitch = (
            _mav_state['lat'],   _mav_state['lon'],
            _mav_state['alt'],   _mav_state['yaw'],
            _mav_state['origin'], _mav_state['roll'], _mav_state['pitch'],
        )

    # Pixel ray in body frame (x=forward, y=right, z=down — NED body convention).
    # The frame is displayed after a 90° CW rotation, so the camera's physical HFOV axis
    # (original width) now spans frame_h pixels — use frame_h for the focal length.
    fx = (frame_h / 2) / math.tan(math.radians(config.CAM_HFOV / 2))
    dx = -(py - frame_h / 2) / fx   # forward (body X)
    dy =  (px - frame_w / 2) / fx   # rightward (body Y)
    dz = 1.0                          # downward (body Z)

    # Rotate body ray to NED earth frame using ZYX Euler (roll → pitch → yaw)
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    # Apply roll around body X
    dx1, dy1, dz1 = dx, cr*dy - sr*dz, sr*dy + cr*dz
    # Apply pitch around body Y
    dx2, dy2, dz2 = cp*dx1 + sp*dz1, dy1, -sp*dx1 + cp*dz1
    # Apply yaw around body Z → NED earth frame
    ned_n, ned_e, ned_d = cy*dx2 - sy*dy2, sy*dx2 + cy*dy2, dz2

    # Convert NED ray to ENU: E=NED-E, N=NED-N, U=-NED-D
    ray_e, ray_n, ray_u = ned_e, ned_n, -ned_d

    # Intersect ray with ground plane (ENU Z=0, drone at Z=alt).
    # Solve: alt + t * ray_u = 0  →  t = -alt / ray_u
    if abs(ray_u) < 1e-6:   # ray nearly horizontal — no valid intersection
        return None
    t = -alt / ray_u        # positive when ray_u < 0 (pointing down)

    east_offset  = t * ray_e
    north_offset = t * ray_n

    # Drone ENU position relative to first GPS fix
    R = 6_371_000.0
    drone_e = math.radians(lon - orig[1]) * R * math.cos(math.radians(orig[0]))
    drone_n = math.radians(lat - orig[0]) * R

    return drone_e + east_offset, drone_n + north_offset


# ---------------------------------------------------------------------------
# Telemetry logger
# ---------------------------------------------------------------------------

_LOG_INTERVAL = 0.5   # seconds between rows (2 Hz)

_STATE_LOG_LABELS = {0: 'IDLE', 1: 'ARMING', 2: 'ARMED', 3: 'FLYING', 4: 'LANDING', 5: 'FAULT'}
_MODE_LOG_LABELS  = {0: 'STAB', 1: 'ALTH',   2: 'POSH',  3: 'AUTO',   4: 'RTH',     5: 'LAND'}

_LOG_HEADER = (
    'time,state,mode,armed,lat,lon,alt_m,roll_deg,pitch_deg,hdg_deg,'
    'spd_ms,climb_ms,batt_pct,volt_v,curr_a,thr_pct,m1_us,m2_us,m3_us,m4_us\n'
)


def start_logging() -> None:
    """Open a timestamped log file in ground/logs/ and start the background writer."""
    log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'logs')
    os.makedirs(log_dir, exist_ok=True)
    fname = datetime.datetime.now().strftime('flight_%Y%m%d_%H%M%S.txt')
    path  = os.path.join(log_dir, fname)
    threading.Thread(target=_log_writer, args=(path,), daemon=True).start()
    print(f"Telemetry log: {path}", flush=True)


def _log_writer(path: str) -> None:
    with open(path, 'w') as f:
        f.write(_LOG_HEADER)
        while True:
            time.sleep(_LOG_INTERVAL)
            with _mav_lock:
                s = dict(_mav_state)

            now       = datetime.datetime.now()
            ts        = now.strftime('%Y-%m-%dT%H:%M:%S.') + f'{now.microsecond // 1000:03d}'
            state_lbl = _STATE_LOG_LABELS.get(s['flight_state'], str(s['flight_state']))
            mode_lbl  = _MODE_LOG_LABELS.get(s['flight_mode_id'], str(s['flight_mode_id']))
            roll_deg  = round(math.degrees(s['roll']),  1)
            pitch_deg = round(math.degrees(s['pitch']), 1)
            hdg_deg   = round(math.degrees(s['yaw']) % 360, 1)
            spd_ms    = round(math.hypot(s['vx'], s['vy']), 2)
            climb_ms  = round(-s['vz'], 2)
            volt_v    = round(s['voltage_mv'] / 1000, 2) if s['voltage_mv'] > 0  else -1.0
            curr_a    = round(s['current_ca'] / 100,   2) if s['current_ca'] >= 0 else -1.0
            lat_s     = f"{s['lat']:.6f}"  if s['lat'] is not None else ''
            lon_s     = f"{s['lon']:.6f}"  if s['lon'] is not None else ''

            row = (f"{ts},{state_lbl},{mode_lbl},"
                   f"{'1' if s['armed'] else '0'},"
                   f"{lat_s},{lon_s},{round(s['alt'], 1)},"
                   f"{roll_deg},{pitch_deg},{hdg_deg},"
                   f"{spd_ms},{climb_ms},"
                   f"{s['battery_pct']},{volt_v},{curr_a},"
                   f"{s['throttle_pct']},"
                   f"{s['motor_pwm'][0]},{s['motor_pwm'][1]},"
                   f"{s['motor_pwm'][2]},{s['motor_pwm'][3]}\n")
            f.write(row)
            f.flush()
