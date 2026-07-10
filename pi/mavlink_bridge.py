#!/usr/bin/env python3
"""
MAVLink bridge: STM32 USART3 (serial) <-> GCS laptop (UDP).
Forwards raw bytes in both directions — no parsing needed here.

Wiring:  Pi GPIO14 (TXD) → STM32 PB11 (RX)
         Pi GPIO15 (RXD) → STM32 PB10 (TX)

Pi UART setup (once, in /boot/config.txt):
    enable_uart=1
    dtoverlay=disable-bt       # frees ttyAMA0 from Bluetooth
Then reboot; /dev/serial0 → /dev/ttyAMA0 at 3V3 logic.

Usage:
    python3 mavlink_bridge.py [LAPTOP_IP]
    LAPTOP_IP env var also accepted (default 192.168.4.2).
"""

import os
import sys
import socket
import threading
import time

SERIAL_PORT = os.environ.get('STM32_PORT', '/dev/serial0')
SERIAL_BAUD = int(os.environ.get('STM32_BAUD', '57600'))
LAPTOP_IP   = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('LAPTOP_IP', '192.168.4.2')
GCS_PORT    = int(os.environ.get('GCS_PORT', '14550'))


def serial_to_udp(ser, sock: socket.socket) -> None:
    while True:
        try:
            data = ser.read(256)
            if data:
                sock.sendto(data, (LAPTOP_IP, GCS_PORT))
        except Exception as e:
            print(f"serial→udp: {e}", flush=True)
            time.sleep(0.1)


_dropped_ips: set = set()


def udp_to_serial(ser, sock: socket.socket) -> None:
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            # Only relay commands from the known GCS laptop (MAVLink injection guard).
            # Announce once per IP so wrong-interface commands don't fail silently.
            if addr[0] != LAPTOP_IP:
                if addr[0] not in _dropped_ips:
                    _dropped_ips.add(addr[0])
                    print(f"udp→serial: ignoring {addr[0]} (allowlist is "
                          f"LAPTOP_IP={LAPTOP_IP}) — fix LAPTOP_IP in ~/pi/.env "
                          f"if that's the laptop", flush=True)
                continue
            ser.write(data)
        except socket.timeout:
            pass
        except Exception as e:
            print(f"udp→serial: {e}", flush=True)
            time.sleep(0.1)


def main() -> None:
    try:
        import serial as _serial
    except ImportError:
        print("pyserial not installed — run: pip install pyserial", flush=True)
        sys.exit(1)

    # write_timeout: a wedged write must error (-> systemd restart), not hang the thread.
    ser = _serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1, write_timeout=0.5)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', GCS_PORT))
    sock.settimeout(1.0)

    print(f"MAVLink bridge: {SERIAL_PORT}@{SERIAL_BAUD} <-> {LAPTOP_IP}:{GCS_PORT}", flush=True)

    threads = [
        threading.Thread(target=serial_to_udp, args=(ser, sock), name='serial→udp', daemon=True),
        threading.Thread(target=udp_to_serial, args=(ser, sock), name='udp→serial', daemon=True),
    ]
    for t in threads:
        t.start()

    try:
        while True:
            time.sleep(1)
            dead = [t.name for t in threads if not t.is_alive()]
            if dead:
                print(f"Thread(s) died: {dead} — exiting for systemd restart", flush=True)
                sys.exit(1)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        sock.close()


if __name__ == '__main__':
    main()
