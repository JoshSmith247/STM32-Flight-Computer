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

# MAVLink v2 constants for the Pi GCS component
PI_SYS_ID        = 10   # distinct from drone's sys_id=1
PI_COMP_ID       = 1
HEARTBEAT_HZ     = 2    # send heartbeat at 2 Hz (generous margin above 3s watchdog)


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


def _heartbeat_sender(ser) -> None:
    """Send MAVLink HEARTBEAT to STM32 at HEARTBEAT_HZ."""
    seq = 0
    interval = 1.0 / HEARTBEAT_HZ
    while True:
        frame = _build_heartbeat(seq)
        try:
            ser.write(frame)
        except Exception as exc:
            print(f"heartbeat write: {exc}", flush=True)
        seq = (seq + 1) & 0xFF
        time.sleep(interval)


def _gcs_to_serial(ser, gcs_port: int) -> None:
    """Forward GCS commands (UDP:GCS_PORT) → STM32 (serial).

    Mirrors the udp_to_serial path from mavlink_bridge.py so arm/disarm,
    mode changes, and mission uploads from the GCS reach the STM32.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', gcs_port))
    sock.settimeout(1.0)
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            ser.write(data)
        except socket.timeout:
            pass
        except Exception as exc:
            print(f"gcs→serial: {exc}", flush=True)


def _weed_target_listener() -> None:
    """Receive weed target positions from the ground station."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', WEED_TARGET_PORT))
    sock.settimeout(1.0)
    print(f"Weed target listener on UDP:{WEED_TARGET_PORT}", flush=True)
    while True:
        try:
            data, addr = sock.recvfrom(256)
            target = json.loads(data.decode())
            wid     = target['wid']
            east_m  = target['east_m']
            north_m = target['north_m']
            print(f"Target W{wid}: {east_m:.2f}m E, {north_m:.2f}m N  (from {addr[0]})",
                  flush=True)
            # TODO: forward as MAVLink SET_POSITION_TARGET_LOCAL_NED to STM32
            # once the STM32 navigation task can parse incoming MAVLink RX commands.
        except socket.timeout:
            pass
        except (json.JSONDecodeError, KeyError) as exc:
            print(f"Bad target packet: {exc}", flush=True)
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

    ser = _serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.05)

    # UDP socket for forwarding STM32 telemetry to the ground station
    gcs_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Weed pilot started", flush=True)
    print(f"  Serial:  {SERIAL_PORT} @ {SERIAL_BAUD} baud", flush=True)
    print(f"  Telem → {LAPTOP_IP}:{GCS_PORT}", flush=True)
    print(f"  GCS  ← UDP:{GCS_PORT}  (commands forwarded to STM32)", flush=True)
    print(f"  HB rate: {HEARTBEAT_HZ} Hz  (watchdog timeout: 3 s)", flush=True)

    threading.Thread(target=_serial_to_udp,       args=(ser, gcs_sock), daemon=True).start()
    threading.Thread(target=_heartbeat_sender,     args=(ser,),          daemon=True).start()
    threading.Thread(target=_gcs_to_serial,        args=(ser, GCS_PORT), daemon=True).start()
    threading.Thread(target=_weed_target_listener,                        daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping.", flush=True)
    finally:
        ser.close()
        gcs_sock.close()


if __name__ == '__main__':
    main()
