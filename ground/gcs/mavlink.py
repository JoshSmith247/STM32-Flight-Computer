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

# Weed target sender - UDP to Pi weed_pilot.py

_target_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# GCS to STM32 command sender: minimal MAVLink v2 COMMAND_LONG frames sent as
# UDP to the Pi's GCS port, relayed to the STM32 by _gcs_to_serial.

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
    # payload: 7xf32 params + u16 cmd + target_sys + target_comp + confirmation
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


# In-air kill switch. The STM32 rejects a normal disarm while Flying/Landing (so a
# stray COMMAND_LONG can't cut motors mid-flight); param2 == this magic overrides
# that guard. On rc-optional builds with no radio wired, this is the ONLY manual
# kill path — the crash cutoff (>75° tilt) and Pi-loss watchdog are the automatic ones.
FORCE_DISARM_MAGIC = 21196.0


def send_emergency_stop() -> None:
    """Force-disarm the drone in ANY state, including in-air — cuts all four motors.

    Sends COMPONENT_ARM_DISARM (400) with param1=0 (disarm) and param2=21196, the
    MAVLink force magic the STM32 requires to honour an in-air disarm.
    """
    print("*** EMERGENCY STOP — force-disarm (param2=21196) sent, motors cut ***", flush=True)
    send_mavlink_command(400, 0.0, FORCE_DISARM_MAGIC)


def send_set_home() -> None:
    """Set home to the current GPS position.

    Sends MAV_CMD_DO_SET_HOME (179, param1=1) to the STM32 and snapshots the
    current lat/lon as the weed-map origin and the current altitude as ground_alt.
    """
    with _mav_lock:
        lat = _mav_state['lat']
        lon = _mav_state['lon']
        alt = _mav_state['alt']
    if lat is None:
        print("set_home: no GPS fix — ignoring", flush=True)
        return
    send_mavlink_command(179, 1.0)   # MAV_CMD_DO_SET_HOME, param1=1 = use current position
    with _mav_lock:
        _mav_state['origin']     = (lat, lon)
        _mav_state['ground_alt'] = alt
        _mav_state['home_set']   = True
    print(f"Home set: {lat:.6f}, {lon:.6f}  ground_alt={alt:.1f} m", flush=True)


# Movement-pad "nudge": uploads a single-waypoint MAVLink mission and relies on
# the STM32's existing mission-upload handshake (telemetry.rs MISSION_COUNT /
# MISSION_ITEM_INT / MISSION_ACK) + Auto-mode sequencer (navigation.rs) to fly
# it. No firmware changes required. CRC_EXTRA values below are copied from
# telemetry.rs's own MAGIC_MISSION_* constants so the two sides agree.

MISSION_COUNT_ID       = 44
MISSION_COUNT_CRC      = 221
MISSION_ITEM_INT_ID    = 73
MISSION_ITEM_INT_CRC   = 38
MISSION_ACK_CRC        = 153   # MISSION_ACK id=47, received only - no sender needed
MAV_CMD_NAV_WAYPOINT   = 16
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 3

_NUDGE_DIRS = {   # (north_m, east_m) unit vectors, NED - matches navigation.rs/estimator.rs
    'N': (1.0, 0.0), 'S': (-1.0, 0.0), 'E': (0.0, 1.0), 'W': (0.0, -1.0),
}

_pending_wp_lock = threading.Lock()
_pending_wp: dict = {'lat_deg': None, 'lon_deg': None, 'alt_m': None}


def _offset_latlon(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple:
    """Flat-earth offset - same approximation as estimator.rs::geo_to_ned / navigation.rs::guide_to."""
    dlat = north_m / 111_320.0
    dlon = east_m / (111_320.0 * math.cos(math.radians(lat_deg)))
    return lat_deg + dlat, lon_deg + dlon


def _build_frame(msgid: int, payload: bytes, crc_extra: int) -> bytes:
    global _cmd_seq
    n   = len(payload)
    seq = _cmd_seq & 0xFF
    _cmd_seq += 1
    header = bytes([n, 0, 0, seq, 255, 0, msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    crc    = _cmd_crc(header + payload, crc_extra)
    return bytes([0xFD]) + header + payload + struct.pack('<H', crc)


def _send_frame(msgid: int, payload: bytes, crc_extra: int) -> None:
    try:
        _cmd_sock.sendto(_build_frame(msgid, payload, crc_extra), (config.PI_IP, config.GCS_PORT))
    except OSError as exc:
        print(f"_send_frame(id={msgid}): {exc}", flush=True)


def _send_mission_item_int(lat_deg: float, lon_deg: float, alt_m: float) -> None:
    """MISSION_ITEM_INT #73, seq=0, MAV_CMD_NAV_WAYPOINT - wire layout matches
    telemetry.rs's parser exactly (params f32x4 @0, x/y i32 @16/20, z f32 @24,
    seq/cmd u16 @28/30); trailing sys/comp/frame/current/autocontinue/type bytes
    are spec-complete but ignored by this firmware."""
    payload = struct.pack(
        '<ffffiifHHBBBBBB',
        0.0, 0.0, 0.0, 0.0,                 # param1..4 (param1 = hold time = fly-through)
        int(round(lat_deg * 1e7)),
        int(round(lon_deg * 1e7)),
        alt_m,
        0, MAV_CMD_NAV_WAYPOINT,             # seq, command
        1, 1,                                # target_system, target_component
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0, 1, 0,                             # current, autocontinue, mission_type
    )
    _send_frame(MISSION_ITEM_INT_ID, payload, MISSION_ITEM_INT_CRC)


def send_nudge_waypoint(direction: str, distance_m: float = 0.5) -> None:
    """Upload a single-waypoint mission `distance_m` away in `direction` (N/E/S/W).

    Only uploads - does NOT switch flight mode. The mission sits in the
    firmware's PENDING_MISSION until Auto mode is selected; warns if it isn't
    already active so a click doesn't silently do nothing.
    """
    if direction not in _NUDGE_DIRS:
        print(f"send_nudge_waypoint: unknown direction {direction!r}", flush=True)
        return
    with _mav_lock:
        lat            = _mav_state['lat']
        lon            = _mav_state['lon']
        alt            = _mav_state['alt']
        flight_mode_id = _mav_state['flight_mode_id']
        armed          = _mav_state['armed']
    if lat is None:
        print("nudge: no GPS fix — ignoring", flush=True)
        return
    n, e = _NUDGE_DIRS[direction]
    target_lat, target_lon = _offset_latlon(lat, lon, n * distance_m, e * distance_m)
    with _pending_wp_lock:
        _pending_wp.update(lat_deg=target_lat, lon_deg=target_lon, alt_m=alt)
    _send_frame(MISSION_COUNT_ID, struct.pack('<HBB', 1, 1, 1), MISSION_COUNT_CRC)
    print(f"Nudge: +{distance_m:.2f} m {direction} → "
          f"{target_lat:.7f}, {target_lon:.7f} (mission upload sent)", flush=True)
    if not (armed and flight_mode_id == 3):
        print("  ⚠ not currently armed+Auto — mission uploaded but won't fly "
              "until Auto mode is selected", flush=True)


def send_follow_target(east_m: float, north_m: float) -> None:
    """Send tracked person's ground position to the Pi as a continuous position setpoint."""
    send_weed_target(0, east_m, north_m)


def send_weed_target(wid: int, east_m: float, north_m: float) -> None:
    """Send selected weed's position to the Pi as NED offsets from the drone's current GPS.

    pixel_to_world() returns absolute ENU from the GPS origin, but the STM32 interprets
    SET_POSITION_TARGET_LOCAL_NED as an offset from the drone's current position.
    Subtract the drone's current ENU position so the Pi forwards correct relative offsets.

    ned_d (down) = drone_agl - EXTRACT_ALT_AGL_M so the STM32 descends to extraction
    altitude.  Clamped to ≥ 0 so we never command an upward jump on a low-flying drone.
    """
    with _mav_lock:
        orig      = _mav_state['origin']
        lat       = _mav_state['lat']
        lon       = _mav_state['lon']
        drone_agl = _mav_state['alt']
    if lat is None or orig is None:
        print(f"W{wid} target not sent: no GPS fix", flush=True)
        return
    R = 6_371_000.0
    drone_e = math.radians(lon - orig[1]) * R * math.cos(math.radians(orig[0]))
    drone_n = math.radians(lat - orig[0]) * R
    rel_e = east_m - drone_e
    rel_n = north_m - drone_n
    ned_d = max(0.0, round(drone_agl - config.EXTRACT_ALT_AGL_M, 3))
    msg = json.dumps({'wid': wid, 'east_m': round(rel_e, 3),
                      'north_m': round(rel_n, 3), 'ned_d': ned_d}).encode()
    try:
        _target_sock.sendto(msg, (config.PI_IP, config.WEED_TARGET_PORT))
        print(f"Sent W{wid} → Pi: {rel_e:+.2f}m E, {rel_n:+.2f}m N, "
              f"ned_d={ned_d:.2f}m  (extract at {config.EXTRACT_ALT_AGL_M}m AGL)", flush=True)
    except OSError as exc:
        print(f"UDP send failed: {exc}", flush=True)


# MAVLink position receiver: GLOBAL_POSITION_INT + ATTITUDE in a background thread.
# pixel_to_world() projects pixel centroids to local ENU metres using the latest fix.

_mav_lock  = threading.Lock()
_mav_state: dict = {
    'lat': None, 'lon': None, 'alt': 1.0, 'yaw': 0.0, 'origin': None,
    'home_set': False, 'ground_alt': 0.0,
    'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'last_msg_t': None,
    'roll': 0.0, 'pitch': 0.0, 'battery_pct': -1,
    'armed': False, 'mode': '',
    'voltage_mv': -1, 'current_ca': -1,
    'mah_consumed': -1, 'time_remaining_s': 0,
    'motor_pwm': [0, 0, 0, 0],
    'mag_x': 0, 'mag_y': 0, 'mag_z': 0,
    # Decoded from HEARTBEAT custom_mode (see telemetry.rs build_heartbeat)
    'flight_mode_id': 0,   # FlightMode enum value [7:0]
    'flight_state':   0,   # FlightState enum value [15:8]
    'payload_flags':  0,   # connected payload bitmask [31:16]
    # GPS_RAW_INT fix quality (0=no GPS, 1=no fix, 2=2D, 3=3D, 4+=DGPS/RTK)
    'gps_fix_type':   0,
    # VFR_HUD collective throttle percentage (0-100)
    'throttle_pct':   0,
    # SYS_STATUS onboard_control_sensors_health bitmask (0 = not yet received)
    'sensors_health': 0,
    # Last COMMAND_ACK received - (cmd_id, result_code); consumed by draw_stats_panel
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
                  'GPS_RAW_INT', 'VFR_HUD', 'SCALED_IMU',
                  'MISSION_REQUEST_INT', 'MISSION_ACK'],
            blocking=True, timeout=1.0)
        if msg is None:
            continue
        with _mav_lock:
            t = time.monotonic()
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = max(0.3, msg.relative_alt / 1000.0)   # mm -> m, clamp
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
            elif msg.get_type() == 'SCALED_IMU':
                _mav_state['mag_x'] = msg.xmag
                _mav_state['mag_y'] = msg.ymag
                _mav_state['mag_z'] = msg.zmag
            elif msg.get_type() == 'COMMAND_ACK':
                # MAV_RESULT enum values
                _RESULT = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
                           3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS'}
                _mav_state['last_ack'] = (msg.command, msg.result)
                if msg.result != 0:
                    print(f"COMMAND_ACK cmd={msg.command} "
                          f"result={_RESULT.get(msg.result, msg.result)}", flush=True)
            elif msg.get_type() == 'MISSION_REQUEST_INT':
                # Firmware requesting item `seq` of the mission upload we started
                # in send_nudge_waypoint(). Only ever a single item (seq=0) today.
                if msg.seq == 0:
                    with _pending_wp_lock:
                        wp = dict(_pending_wp)
                    if wp['lat_deg'] is not None:
                        _send_mission_item_int(wp['lat_deg'], wp['lon_deg'], wp['alt_m'])
            elif msg.get_type() == 'MISSION_ACK':
                with _pending_wp_lock:
                    _pending_wp.update(lat_deg=None, lon_deg=None, alt_m=None)
                if msg.type != 0:   # MAV_MISSION_ACCEPTED = 0
                    print(f"Mission upload rejected: type={msg.type}", flush=True)
            if config.VERBOSE_LOGGING:
                mt = msg.get_type()
                if mt == 'HEARTBEAT':
                    print(f"[MAV] HB  armed={_mav_state['armed']}"
                          f"  state={_mav_state['flight_state']}"
                          f"  mode={_mav_state['flight_mode_id']}", flush=True)
                elif mt == 'GPS_RAW_INT':
                    print(f"[MAV] GPS fix={_mav_state['gps_fix_type']}", flush=True)
                elif mt == 'SYS_STATUS':
                    print(f"[MAV] SYS health=0x{_mav_state['sensors_health']:04X}"
                          f"  batt={_mav_state['battery_pct']}%", flush=True)
                elif mt == 'BATTERY_STATUS':
                    v = _mav_state['voltage_mv']
                    print(f"[MAV] BAT  {v/1000:.2f}V"
                          f"  {_mav_state['current_ca']/100:.1f}A"
                          f"  {_mav_state['battery_pct']}%"
                          f"  mah={_mav_state['mah_consumed']}", flush=True)
                elif mt == 'SERVO_OUTPUT_RAW':
                    print(f"[MAV] MOTORS {_mav_state['motor_pwm']}", flush=True)
                elif mt == 'COMMAND_ACK':
                    print(f"[MAV] ACK  cmd={msg.command}"
                          f"  {_RESULT.get(msg.result, msg.result)}", flush=True)


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

    # Pixel ray in NED body frame. The frame is displayed after a 90-deg CW rotation,
    # so the camera's physical HFOV axis spans frame_h pixels - use frame_h for focal length.
    fx = (frame_h / 2) / math.tan(math.radians(config.CAM_HFOV / 2))
    dx = -(py - frame_h / 2) / fx   # forward (body X)
    dy =  (px - frame_w / 2) / fx   # rightward (body Y)
    dz = 1.0                          # downward (body Z)

    # Rotate body ray to NED earth frame using ZYX Euler (roll -> pitch -> yaw)
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    # Apply roll around body X
    dx1, dy1, dz1 = dx, cr*dy - sr*dz, sr*dy + cr*dz
    # Apply pitch around body Y
    dx2, dy2, dz2 = cp*dx1 + sp*dz1, dy1, -sp*dx1 + cp*dz1
    # Apply yaw around body Z -> NED earth frame
    ned_n, ned_e, ned_d = cy*dx2 - sy*dy2, sy*dx2 + cy*dy2, dz2

    # Convert NED ray to ENU: E=NED-E, N=NED-N, U=-NED-D
    ray_e, ray_n, ray_u = ned_e, ned_n, -ned_d

    # Intersect ray with ground plane (ENU Z=0, drone at Z=alt).
    # Solve: alt + t * ray_u = 0 -> t = -alt / ray_u
    if abs(ray_u) < 1e-6:   # ray nearly horizontal - no valid intersection
        return None
    t = -alt / ray_u        # positive when ray_u < 0 (pointing down)

    east_offset  = t * ray_e
    north_offset = t * ray_n

    # Drone ENU position relative to first GPS fix
    R = 6_371_000.0
    drone_e = math.radians(lon - orig[1]) * R * math.cos(math.radians(orig[0]))
    drone_n = math.radians(lat - orig[0]) * R

    return drone_e + east_offset, drone_n + north_offset


# Telemetry logger

_LOG_INTERVAL = 0.5   # seconds between rows (2 Hz)

_STATE_LOG_LABELS = {0: 'IDLE', 1: 'ARMING', 2: 'ARMED', 3: 'FLYING', 4: 'LANDING', 5: 'FAULT'}
_MODE_LOG_LABELS  = {0: 'STAB', 1: 'ALTH',   2: 'POSH',  3: 'AUTO',   4: 'RTH',     5: 'LAND', 6: 'FOLW'}

_LOG_HEADER = (
    'time,state,mode,armed,lat,lon,alt_m,roll_deg,pitch_deg,hdg_deg,'
    'spd_ms,climb_ms,batt_pct,volt_v,curr_a,thr_pct,m1_us,m2_us,m3_us,m4_us,'
    'mag_x,mag_y,mag_z\n'
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
                   f"{s['motor_pwm'][2]},{s['motor_pwm'][3]},"
                   f"{s['mag_x']},{s['mag_y']},{s['mag_z']}\n")
            f.write(row)
            f.flush()
