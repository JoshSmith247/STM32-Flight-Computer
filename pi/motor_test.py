#!/usr/bin/env python3
"""
Bench motor test — fire MAV_CMD_DO_MOTOR_TEST (209) at the flight controller.

⚠ PROPS OFF. The firmware spins ONE motor at low throttle for 2 s, then auto-stops.
It honours the command ONLY while disarmed and on the ground (FlightState::Idle),
and clamps the throttle to 0.20 — see telemetry.rs handle_command(209) / motor.rs.

Runs on whatever machine is wired to the FC's MAVLink serial link:
  • Bench over the Nucleo ST-LINK VCP: flash with `cargo run --features nucleo-vcp`
    (routes USART3 → PD8/PD9), then point --port at the VCP:
        macOS: /dev/tty.usbmodem*      Linux: /dev/ttyACM*
  • On the Pi (deployed PB10/PB11 wiring): --port /dev/serial0.

Firmware's DO_MOTOR_TEST param convention (NOT the stock MAVLink one):
    param1 = motor index 1..4   (firmware M1..M4)
    param2 = throttle 0.0..1.0  (clamped to 0.20)

A background thread streams GCS HEARTBEATs at 4 Hz the whole time the tool runs,
so the firmware's Pi-loss watchdog (5 s timeout) never trips a Fault — a Fault
would make the board reject the test.

Install:  pip install pymavlink

Usage:
    python3 motor_test.py -m 1                       # M1 @ default 0.08, autodetect port
    python3 motor_test.py -m 3 -t 0.10 -p /dev/tty.usbmodem1403
    python3 motor_test.py -m 2 -p /dev/serial0       # from the Pi
"""

import argparse
import glob
import os
import sys
import threading
import time

# The flight-controller RX parser is MAVLink v2 ONLY (it syncs on 0xFD). pymavlink
# can default to v1 (0xFE), which the FC silently ignores → no ACK. Force v2 here,
# BEFORE importing pymavlink (the env var is read at import time).
os.environ.setdefault('MAVLINK20', '1')

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink not installed — run: pip install pymavlink")

THROTTLE_MAX = 0.40   # TEMP bench diagnostic: was 0.20 — revert with telemetry.rs clamp
HEARTBEAT_HZ = 4      # < the FC's 5 s Pi-loss watchdog timeout


def autodetect_port() -> str | None:
    """Best-effort guess at the FC serial port for bench use (ST-LINK VCP)."""
    candidates = sorted(glob.glob('/dev/tty.usbmodem*')   # macOS
                        + glob.glob('/dev/ttyACM*'))       # Linux
    return candidates[0] if candidates else None


def heartbeat_loop(master, stop: threading.Event, lock: threading.Lock) -> None:
    """Stream GCS HEARTBEATs so the FC's Pi-loss watchdog stays satisfied."""
    interval = 1.0 / HEARTBEAT_HZ
    while not stop.is_set():
        with lock:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0)
        time.sleep(interval)


def main() -> None:
    ap = argparse.ArgumentParser(description="Bench motor test (PROPS OFF).")
    ap.add_argument('-m', '--motor', type=int, choices=(1, 2, 3, 4),
                    help="motor index 1..4 (firmware M1..M4); required unless --listen")
    ap.add_argument('-t', '--throttle', type=float, default=0.08,
                    help=f"throttle 0.0..{THROTTLE_MAX} (default 0.08)")
    ap.add_argument('-p', '--port', default=None,
                    help="serial port (default: autodetect ST-LINK VCP)")
    ap.add_argument('-b', '--baud', type=int, default=57600,
                    help="baud (default 57600, matches telemetry_task)")
    ap.add_argument('--listen', action='store_true',
                    help="link diagnostic: receive + count frames FROM the FC for 6 s, "
                         "send nothing, then exit (no motor command)")
    args = ap.parse_args()

    if not 0.0 <= args.throttle <= THROTTLE_MAX:
        ap.error(f"throttle must be within 0.0..{THROTTLE_MAX} (firmware safety clamp)")
    if not args.listen and args.motor is None:
        ap.error("--motor is required (or use --listen for the link diagnostic)")

    port = args.port or autodetect_port()
    if not port:
        sys.exit("No serial port found — pass --port (macOS /dev/tty.usbmodem*, "
                 "Linux /dev/ttyACM* for the VCP, or /dev/serial0 on the Pi).")

    print(f"Connecting to {port} @ {args.baud}…")
    master = mavutil.mavlink_connection(port, baud=args.baud,
                                        source_system=10, source_component=1)

    # Link diagnostic: just read frames coming FROM the FC. FC→host is whole-frame
    # DMA, so a healthy count here while host→FC commands fail isolates the fault to
    # the FC's RX path (dropped bytes) rather than baud/link.
    if args.listen:
        print("Listening 6 s for frames from the FC (sending nothing)…")
        good = 0
        t = time.time()
        while time.time() - t < 6.0:
            msg = master.recv_match(blocking=True, timeout=0.5)
            if msg is not None:
                good += 1
                print(f"  decoded: {msg.get_type()} (sys {msg.get_srcSystem()})")
        print(f"\ntotal good frames from FC: {good}")
        return

    # One lock guards every send: the heartbeat thread and the command below
    # both write through master.mav, and interleaved byte streams corrupt frames.
    send_lock = threading.Lock()
    stop = threading.Event()
    hb = threading.Thread(target=heartbeat_loop, args=(master, stop, send_lock),
                          name='heartbeat', daemon=True)
    hb.start()
    time.sleep(0.5)   # let a couple of heartbeats land before commanding

    print(f"⚠ PROPS OFF — spinning M{args.motor} @ {args.throttle:.2f} for 2 s")
    with send_lock:
        master.mav.command_long_send(
            1, 1,                                    # target system, component
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,   # 209
            0,                                       # confirmation
            float(args.motor),                       # param1 = motor index
            float(args.throttle),                    # param2 = throttle
            0, 0, 0, 0, 0)                           # param3..7 unused

    # Wait for the COMMAND_ACK (msg 77) the firmware sends back.
    result = None
    deadline = time.time() + 3.0
    while time.time() < deadline:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
            result = msg.result
            name = mavutil.mavlink.enums['MAV_RESULT'][result].name
            print(f"ACK: {name}")
            if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("  Rejected — check: disarmed + FlightState::Idle (not Fault), "
                      "motor index 1..4, and that the FC is receiving our heartbeats.")
            break

    if result is None:
        print("No COMMAND_ACK received — is --port the MAVLink link, baud 57600, "
              "and the FC running?")
    elif result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        # Keep heartbeating through the 2 s spin so the watchdog never trips mid-test.
        time.sleep(2.2)

    stop.set()
    print("Done.")


if __name__ == '__main__':
    main()
