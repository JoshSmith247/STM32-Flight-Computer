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
    python3 motor_test.py --sequence                 # all 4 in order: motor ORDER + DIRECTION check
    python3 motor_test.py --self-level               # ⚠ ARMS (props off): tilt frame, watch outputs

--self-level needs a firmware built with rc-optional (no RC on the bench) and,
with no battery divider wired, no-batt: `--features nucleo-vcp,rc-optional,no-batt`.
"""

import argparse
import glob
import os
import sys
import threading
import time

# The FC RX parser is MAVLink v2 ONLY; pymavlink can default to v1, which the FC
# silently ignores. Force v2 BEFORE importing pymavlink (env var read at import).
os.environ.setdefault('MAVLINK20', '1')

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink not installed — run: pip install pymavlink")

THROTTLE_MAX = 0.20   # matches the firmware clamp in telemetry.rs (cmd 209)
HEARTBEAT_HZ = 4      # < the FC's 5 s Pi-loss watchdog timeout

# Quad-X layout per the firmware mixer (pid.rs), viewed from above:
# front M2(CCW) M1(CW), back M4(CW) M3(CCW).
MOTOR_POSITIONS = {
    1: ("FRONT-RIGHT", "CW"),
    2: ("FRONT-LEFT",  "CCW"),
    3: ("BACK-RIGHT",  "CCW"),
    4: ("BACK-LEFT",   "CW"),
}


def autodetect_port() -> str | None:
    """Best-effort guess at the FC serial port for bench use (ST-LINK VCP)."""
    candidates = sorted(glob.glob('/dev/tty.usbmodem*')   # macOS
                        + glob.glob('/dev/ttyACM*'))       # Linux
    return candidates[0] if candidates else None


def spin_and_ack(master, lock: threading.Lock, motor: int, throttle: float) -> bool:
    """Send DO_MOTOR_TEST for one motor and wait for the ACK. True = accepted."""
    with lock:
        master.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST, 0,
            float(motor), float(throttle), 0, 0, 0, 0, 0)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
            ok = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            if not ok:
                name = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
                print(f"  M{motor} REJECTED ({name}) — FC must be disarmed + Idle (not Fault)")
            return ok
    print(f"  M{motor}: no ACK — link/baud/FC-running?")
    return False


def run_sequence(master, lock: threading.Lock, throttle: float) -> None:
    """Motor ORDER + DIRECTION check: spin M1..M4 one at a time, record a
    per-motor verdict, and print a pass/fail scoreboard with exact fixes."""
    print("\n⚠ PROPS OFF — motor order + direction check.")
    print("Orient the frame so you know which corner is FRONT, viewed from above.")
    print("Direction tip: a bit of tape on the motor bell makes the spin obvious.\n")
    input("Props removed and frame secured? Press Enter to start… ")

    results = {}
    for m in (1, 2, 3, 4):
        pos, rot = MOTOR_POSITIONS[m]
        while True:
            print(f"\n→ M{m}: expect the {pos} motor spinning {rot} (viewed from above), 2 s…")
            if not spin_and_ack(master, lock, m, throttle):
                results[m] = "no spin / rejected"
                break
            time.sleep(2.6)  # let the 2 s spin finish + margin
            ans = input(f"  Verdict — [Enter]={pos} spun {rot} ✓   [c]=wrong corner   "
                        f"[d]=right corner, wrong direction   [r]=repeat spin: ").strip().lower()
            if ans == 'r':
                continue
            results[m] = {'': 'ok', 'y': 'ok',
                          'c': 'WRONG CORNER',
                          'd': 'WRONG DIRECTION'}.get(ans, f"unrecognised: {ans!r}")
            break

    print("\n── Scoreboard ──")
    all_ok = True
    for m in (1, 2, 3, 4):
        pos, rot = MOTOR_POSITIONS[m]
        res = results.get(m, '?')
        ok = res == 'ok'
        all_ok &= ok
        print(f"  {'✓' if ok else '✗'}  M{m}  {pos:<11s} {rot:<3s} — {'verified' if ok else res}")
    if all_ok:
        print("\nALL FOUR VERIFIED — motor order AND direction are props-on ready.")
    else:
        print("\nFixes, then RE-RUN until all four verify:")
        print("  WRONG CORNER     → update MOTOR_SLOT in src/actuators/motor.rs to match")
        print("                     (or physically swap those ESC signal wires).")
        print("  WRONG DIRECTION  → flip that motor in BLHeli configurator, or swap any")
        print("                     two of its three phase wires at the ESC.")


def run_self_level(master, lock: threading.Lock) -> None:
    """⚠ ARMS the craft (motors at 4 % idle). Live motor outputs + attitude so
    tilting the frame shows the control loop responding. Ctrl-C disarms."""
    print("\n⚠⚠ SELF-LEVEL CHECK — this ARMS the flight controller.")
    print("   All four motors will spin at the 4 % idle floor the moment it arms.")
    if input("   Type PROPS OFF to confirm: ").strip().upper() != "PROPS OFF":
        print("   Not confirmed — aborting.")
        return

    with lock:
        master.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
    deadline = time.time() + 3.0
    armed = False
    while time.time() < deadline:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            armed = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            break
    if not armed:
        print("Arm REJECTED or no ACK. Checklist: firmware built with rc-optional")
        print("(+ no-batt if no divider), IMU healthy, not Fault. See defmt/journal for the reason.")
        return

    print("ARMED — motors at idle. Tilt the frame and watch which outputs rise:")
    print("  tilt RIGHT  → M1 + M3 rise      tilt NOSE-DOWN → M1 + M2 rise")
    print("Ctrl-C to disarm.\n")
    r2d = 57.2958
    m = [0, 0, 0, 0]
    roll = pitch = 0.0
    try:
        while True:
            msg = master.recv_match(type=['SERVO_OUTPUT_RAW', 'ATTITUDE'],
                                    blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.get_type() == 'ATTITUDE':
                roll, pitch = msg.roll * r2d, msg.pitch * r2d
            else:
                m = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
            print(f"\r  M1 {m[0]:4d}  M2 {m[1]:4d}  M3 {m[2]:4d}  M4 {m[3]:4d} µs"
                  f"  |  roll {roll:+6.1f}°  pitch {pitch:+6.1f}°   ",
                  end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        with lock:
            master.mav.command_long_send(
                1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 0, 0, 0, 0, 0, 0)
        print("\nDisarm sent.")


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
    ap.add_argument('--sequence', action='store_true',
                    help="⚠ PROPS OFF: spin M1..M4 one at a time with position/direction "
                         "prompts — the motor ORDER + DIRECTION acceptance check")
    ap.add_argument('--self-level', dest='self_level', action='store_true',
                    help="⚠⚠ PROPS OFF, ARMS THE FC: motors at 4%% idle, live "
                         "SERVO_OUTPUT_RAW + ATTITUDE readout while you tilt the frame; "
                         "Ctrl-C disarms (needs rc-optional firmware)")
    args = ap.parse_args()

    if not 0.0 <= args.throttle <= THROTTLE_MAX:
        ap.error(f"throttle must be within 0.0..{THROTTLE_MAX} (firmware safety clamp)")
    if not (args.listen or args.sequence or args.self_level) and args.motor is None:
        ap.error("--motor is required (or use --listen / --sequence / --self-level)")

    port = args.port or autodetect_port()
    if not port:
        sys.exit("No serial port found — pass --port (macOS /dev/tty.usbmodem*, "
                 "Linux /dev/ttyACM* for the VCP, or /dev/serial0 on the Pi).")

    print(f"Connecting to {port} @ {args.baud}…")
    master = mavutil.mavlink_connection(port, baud=args.baud,
                                        source_system=10, source_component=1)

    # Link diagnostic: just read frames coming FROM the FC. FC->host is whole-frame
    # DMA, so a healthy count here while host->FC commands fail isolates the fault to
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

    if args.sequence:
        try:
            run_sequence(master, send_lock, args.throttle)
        finally:
            stop.set()
        return

    if args.self_level:
        try:
            run_self_level(master, send_lock)
        finally:
            stop.set()
        return

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
