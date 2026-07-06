#!/usr/bin/env python3
"""
Weed pilot — Pi-side relay between ground station weed detection and STM32.

Replaces mavlink_bridge.py when weed targeting is active. Runs on the Pi.

Responsibilities:
  1. Forward STM32 telemetry bytes (serial) → ground station laptop (UDP:GCS_PORT)
     so the ground station MAVLink listener can read ATTITUDE / GLOBAL_POSITION_INT.
  2. Send a periodic MAVLink v2 HEARTBEAT to the STM32 over serial so the
     STM32 heartbeat watchdog in telemetry_task stays happy.
  3. Listen for weed target positions from the ground station (UDP:WEED_TARGET_PORT)
     and log them.  Future: translate to MAVLink position commands for the STM32.

Wiring (Pi ↔ STM32):
  Pi GPIO14 (TXD, /dev/serial0) → STM32 PB11 (USART3 RX)
  Pi GPIO15 (RXD, /dev/serial0) → STM32 PB10 (USART3 TX)

Pi UART setup (once, in /boot/config.txt or /boot/firmware/config.txt):
  enable_uart=1
  dtoverlay=disable-bt    # frees ttyAMA0 from Bluetooth; /dev/serial0 → ttyAMA0
Then reboot.

Usage:
  python3 weed_pilot.py [LAPTOP_IP]
  Or set env vars: LAPTOP_IP, PI_IP, STM32_PORT, STM32_BAUD, GCS_PORT, WEED_TARGET_PORT

Dependencies:
  pip install pyserial
"""

import os
import sys
import json
import socket
import struct
import threading
import time

SERIAL_PORT      = os.environ.get('STM32_PORT',        '/dev/serial0')
SERIAL_BAUD      = int(os.environ.get('STM32_BAUD',    '57600'))
LAPTOP_IP        = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('LAPTOP_IP', '192.168.4.2')
GCS_PORT         = int(os.environ.get('GCS_PORT',        '14550'))
WEED_TARGET_PORT = int(os.environ.get('WEED_TARGET_PORT', '5700'))
# Event log lives in /tmp with a FIXED name, so it can't fill the SD card and a
# crash-loop won't spawn a new file every 3 s. /tmp is cleared on reboot.
# Override with WEED_LOG=/dev/shm/... for guaranteed RAM-backing.
WEED_LOG_PATH    = os.environ.get('WEED_LOG', '/tmp/weed_events.jsonl')

# MAVLink v2 constants for the Pi GCS component
PI_SYS_ID        = 10   # distinct from drone's sys_id=1
PI_COMP_ID       = 1
HEARTBEAT_HZ     = 4    # send heartbeat at 4 Hz — many retries within the FC's 5 s Pi-loss watchdog

# Shared monotonic sequence number for all frames sent from PI_SYS_ID/PI_COMP_ID.
# Lock required: heartbeat and weed-target threads both increment it.
_seq_lock = threading.Lock()
_seq      = 0

# Reference point for time_boot_ms (ms since this process started).
_BOOT_TIME = time.monotonic()


def _next_seq() -> int:
    global _seq
    with _seq_lock:
        s = _seq
        _seq = (s + 1) & 0xFF
        return s


def _time_boot_ms() -> int:
    return int((time.monotonic() - _BOOT_TIME) * 1000) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# MAVLink v2 frame builder (minimal — HEARTBEAT only)
# ---------------------------------------------------------------------------

def _mavlink_crc(data: bytes, extra: int) -> int:
    crc = 0xFFFF
    for byte in list(data) + [extra]:
        tmp = (byte ^ (crc & 0xFF)) & 0xFF
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def _build_set_position_target_local_ned(seq: int, north_m: float, east_m: float,
                                         down_m: float = 0.0) -> bytes:
    """Build a MAVLink v2 SET_POSITION_TARGET_LOCAL_NED frame (msg_id=84, CRC_EXTRA=143).

    Position-only command (type_mask=0x0FF8) in MAV_FRAME_LOCAL_OFFSET_NED
    (frame=7): north/east/down OFFSETS from the drone's current position —
    exactly how the STM32 interprets them. down_m > 0 commands a descent.
    """
    # MAVLink v2 wire format: fields sorted largest-first (all floats before u16/u8)
    payload = struct.pack(
        '<IfffffffffffHBBB',
        _time_boot_ms(),                                    # time_boot_ms          @0
        float(north_m), float(east_m), float(down_m),      # x(N), y(E), z(D)     @4,8,12
        0.0, 0.0, 0.0,                                      # vx, vy, vz (ignored)  @16,20,24
        0.0, 0.0, 0.0,                                      # afx, afy, afz         @28,32,36
        0.0, 0.0,                                           # yaw, yaw_rate         @40,44
        0x0FF8,                                             # type_mask: pos only   @48
        1,                                                  # target_system         @50
        1,                                                  # target_component      @51
        7,                                                  # MAV_FRAME_LOCAL_OFFSET_NED @52
    )
    n = len(payload)  # 53
    header = bytes([n, 0, 0, seq & 0xFF, PI_SYS_ID, PI_COMP_ID,
                    84 & 0xFF, (84 >> 8) & 0xFF, (84 >> 16) & 0xFF])
    crc = _mavlink_crc(header + payload, 143)
    return bytes([0xFD]) + header + payload + struct.pack('<H', crc)


def _build_heartbeat(seq: int) -> bytes:
    """Build a MAVLink v2 HEARTBEAT frame (msg_id=0, CRC_EXTRA=50)."""
    payload = bytes([
        0, 0, 0, 0,  # custom_mode (4 bytes)
        6,           # MAV_TYPE_GCS
        0,           # MAV_AUTOPILOT_INVALID
        0,           # base_mode
        4,           # MAV_STATE_ACTIVE
        3,           # MAVLink version
    ])
    n = len(payload)                              # 9
    header = bytes([n, 0, 0, seq & 0xFF, PI_SYS_ID, PI_COMP_ID, 0, 0, 0])
    crc = _mavlink_crc(header + payload, 50)
    return bytes([0xFD]) + header + payload + struct.pack('<H', crc)


# ---------------------------------------------------------------------------
# Event log — JSONL, one record per line for easy grep / replay
# ---------------------------------------------------------------------------

_log_lock = threading.Lock()


def _log_event(**kw: object) -> None:
    entry = json.dumps({'ts': time.time(), **kw})
    try:
        with _log_lock:
            with open(WEED_LOG_PATH, 'a') as f:
                f.write(entry + '\n')
    except Exception as exc:
        print(f"log write: {exc}", flush=True)


# ---------------------------------------------------------------------------
# Threads
# ---------------------------------------------------------------------------

def _serial_to_udp(ser, sock: socket.socket) -> None:
    """Forward raw telemetry bytes from STM32 → ground station."""
    while True:
        try:
            data = ser.read(256)
            if data:
                sock.sendto(data, (LAPTOP_IP, GCS_PORT))
        except Exception as exc:
            print(f"serial→udp: {exc}", flush=True)
            time.sleep(0.1)


def _heartbeat_sender(ser, ser_lock: threading.Lock) -> None:
    """Send MAVLink HEARTBEAT to STM32 at HEARTBEAT_HZ."""
    interval = 1.0 / HEARTBEAT_HZ
    while True:
        frame = _build_heartbeat(_next_seq())
        try:
            with ser_lock:
                ser.write(frame)
        except Exception as exc:
            print(f"heartbeat write: {exc}", flush=True)
        time.sleep(interval)


def _gcs_to_serial(ser, gcs_port: int, ser_lock: threading.Lock) -> None:
    """Forward GCS commands (UDP:GCS_PORT) → STM32 (serial).

    Mirrors the udp_to_serial path from mavlink_bridge.py so arm/disarm,
    mode changes, and mission uploads from the GCS reach the STM32.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', gcs_port))
    sock.settimeout(1.0)
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            if addr[0] != LAPTOP_IP:
                continue
            with ser_lock:
                ser.write(data)
        except socket.timeout:
            pass
        except Exception as exc:
            print(f"gcs→serial: {exc}", flush=True)


def _weed_target_listener(ser, ser_lock: threading.Lock) -> None:
    """Receive weed target positions from the GCS and forward to STM32 as
    SET_POSITION_TARGET_LOCAL_NED (msg 84) so navigation_task can fly to them."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', WEED_TARGET_PORT))
    sock.settimeout(1.0)
    print(f"Weed target listener on UDP:{WEED_TARGET_PORT}", flush=True)
    while True:
        try:
            data, addr = sock.recvfrom(256)
            if addr[0] != LAPTOP_IP:
                continue
            target  = json.loads(data.decode())
            wid     = target['wid']
            east_m  = float(target['east_m'])
            north_m = float(target['north_m'])
            down_m  = float(target.get('ned_d', 0.0))
            print(f"Target W{wid}: {east_m:.2f}m E, {north_m:.2f}m N, "
                  f"ned_d={down_m:.2f}m  (from {addr[0]})", flush=True)
            _log_event(event='target_received', wid=wid, east_m=east_m, north_m=north_m,
                       ned_d=down_m, from_ip=addr[0])
            frame = _build_set_position_target_local_ned(_next_seq(), north_m, east_m, down_m)
            with ser_lock:
                ser.write(frame)
            print(f"  → forwarded W{wid} to STM32 as SET_POSITION_TARGET_LOCAL_NED",
                  flush=True)
            _log_event(event='target_forwarded', wid=wid, east_m=east_m, north_m=north_m,
                       ned_d=down_m)
        except socket.timeout:
            pass
        except (json.JSONDecodeError, KeyError) as exc:
            print(f"Bad target packet: {exc}", flush=True)
            _log_event(event='bad_packet', error=str(exc))
        except Exception as exc:
            print(f"weed listener: {exc}", flush=True)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    try:
        import serial as _serial
    except ImportError:
        print("pyserial not installed — run: pip install pyserial", flush=True)
        sys.exit(1)

    try:
        ser = _serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.05, write_timeout=0.5)
    except _serial.SerialException as exc:
        print(f"Could not open serial {SERIAL_PORT}: {exc}", flush=True)
        print("  Is the UART enabled? Needs enable_uart=1 + dtoverlay=disable-bt in "
              "/boot/firmware/config.txt, then a reboot.", flush=True)
        time.sleep(10)   # back off so systemd doesn't tight-loop on a config error
        sys.exit(1)

    # UDP socket for forwarding STM32 telemetry to the ground station
    gcs_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Weed pilot started", flush=True)
    print(f"  Serial:  {SERIAL_PORT} @ {SERIAL_BAUD} baud", flush=True)
    print(f"  Telem → {LAPTOP_IP}:{GCS_PORT}", flush=True)
    print(f"  GCS  ← UDP:{GCS_PORT}  (commands forwarded to STM32)", flush=True)
    print(f"  HB rate: {HEARTBEAT_HZ} Hz  (watchdog timeout: 5 s)", flush=True)
    print(f"  Event log: {WEED_LOG_PATH}", flush=True)
    _log_event(event='pilot_started', serial=SERIAL_PORT, baud=SERIAL_BAUD,
               laptop_ip=LAPTOP_IP, gcs_port=GCS_PORT)

    ser_lock = threading.Lock()

    threads = [
        threading.Thread(target=_serial_to_udp,       args=(ser, gcs_sock),           name='serial→udp',    daemon=True),
        threading.Thread(target=_heartbeat_sender,     args=(ser, ser_lock),           name='heartbeat',     daemon=True),
        threading.Thread(target=_gcs_to_serial,        args=(ser, GCS_PORT, ser_lock), name='gcs→serial',    daemon=True),
        threading.Thread(target=_weed_target_listener, args=(ser, ser_lock),           name='weed-listener', daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        while True:
            time.sleep(1)
            dead = [t.name for t in threads if not t.is_alive()]
            if dead:
                print(f"Thread(s) died: {dead} — exiting for systemd restart", flush=True)
                _log_event(event='thread_died', threads=dead)
                sys.exit(1)
    except KeyboardInterrupt:
        print("Stopping.", flush=True)
    finally:
        ser.close()
        gcs_sock.close()


if __name__ == '__main__':
    main()
