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
import serial

SERIAL_PORT = os.environ.get('STM32_PORT', '/dev/serial0')
SERIAL_BAUD = int(os.environ.get('STM32_BAUD', '57600'))
LAPTOP_IP   = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('LAPTOP_IP', '192.168.4.2')
GCS_PORT    = int(os.environ.get('GCS_PORT', '14550'))


def serial_to_udp(ser: serial.Serial, sock: socket.socket) -> None:
    while True:
        try:
            data = ser.read(256)
            if data:
                sock.sendto(data, (LAPTOP_IP, GCS_PORT))
        except Exception as e:
            print(f"serial→udp: {e}", flush=True)
            time.sleep(0.1)


def udp_to_serial(ser: serial.Serial, sock: socket.socket) -> None:
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            ser.write(data)
        except socket.timeout:
            pass
        except Exception as e:
            print(f"udp→serial: {e}", flush=True)
            time.sleep(0.1)


def main() -> None:
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', GCS_PORT))
    sock.settimeout(1.0)

    print(f"MAVLink bridge: {SERIAL_PORT}@{SERIAL_BAUD} <-> {LAPTOP_IP}:{GCS_PORT}", flush=True)

    threading.Thread(target=serial_to_udp, args=(ser, sock), daemon=True).start()
    threading.Thread(target=udp_to_serial, args=(ser, sock), daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        ser.close()
        sock.close()


if __name__ == '__main__':
    main()
