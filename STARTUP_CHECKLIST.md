# Startup Checklist

End-to-end procedure from power-off to armed and flying.  
Steps are ordered; each section depends on the previous one completing successfully.

> **Current build state (2026-08-29).** All cargo features have been removed — there is one
> build, targeting the Nucleo pinout (MOSI **PD7**, LED **PE1**). **No radio is fitted**, so the
> RC gates stand down and GCS disarm is the only manual kill path. `SAFETY_PIN_ENABLED = false`,
> so the remove-before-flight pin is **not checked**. Sections below are annotated where this
> changes the procedure.

---

## 0 · One-time setup (do once per build, not every flight)

These are prerequisites — if already done, skip to Section 1.

- [ ] **Flash firmware** — `cargo run --release`. No feature flags: the Nucleo pinout is
      unconditional and probe-rs boots it directly. MAVLink is on USART3 → Pi header pins
      (PB10/PB11). On a **custom FC** (DFU): true power-cycle after every flash — CR3 is
      write-once per POR and a soft reset hangs the time driver (board looks dead)
- [ ] **Wire the safety inputs** — remove-before-flight jumper header on **PA0↔GND**.
      ⚠ `SAFETY_PIN_ENABLED = false` in `src/main.rs:53`, so the firmware currently **ignores**
      this pin. Set it `true` before any field flight.
      (PC2 grip microswitch is unused — `grip-sense` was removed)
- [ ] **Verify voltage divider** — measure R1 and R2 on the battery sense line and confirm `V_DIVIDER = (R1 + R2) / R2` in `src/sensors/battery.rs` matches
- [ ] **Current sense (optional)** — ESC CUR pad → PF3, calibrate `CUR_A_PER_V` against a
      clamp meter, then set `CUR_SENSE_FITTED = true` in `src/sensors/battery.rs`
- [ ] **Pi services installed** — run the setup block from `pi/README.md` on the Pi once:
  ```bash
  sed -i 's|/home/pi/drone|/home/jsmith/pi|g; s|User=pi|User=jsmith|g' \
      ~/pi/mavlink.service ~/pi/camera.service
  sudo cp ~/pi/mavlink.service ~/pi/camera.service /etc/systemd/system/
  sudo systemctl daemon-reload
  sudo systemctl enable mavlink camera
  ```
- [ ] **Pi `.env` configured** — copy `.env.example` → `.env`, set `LAPTOP_IP` to your laptop's hotspot IP (find it with `ipconfig getifaddr en0`)
- [ ] **ELRS receiver in SBUS mode** — confirm the BETAFPV ELRS Lite is configured for SBUS output, not CRSF
- [ ] **Receiver failsafe mode set to *no output* or *preset positions*** — ⚠ **never
      "hold last values."** The firmware detects link loss four ways, and every one of them
      depends on the receiver either setting the SBUS failsafe flag or going silent. A
      receiver configured to repeat its last stick positions produces well-formed, correctly
      timed, unflagged frames forever — an invisible failure the firmware cannot detect.
- [ ] **GCS dependencies installed** — `cd ground && pip install -r requirements.txt`

---

## 1 · Pre-flight hardware check

- [ ] Props secure (Loctite cured, correct rotation per motor — CW: M1/M4, CCW: M2/M3)
- [ ] All four motor screws tight
- [ ] Battery connector seated and latched, balance lead connected to checker
- [ ] Lipo voltage reads ≥ 15.2 V (4S, ~3.8 V/cell resting) — below 14.8 V, charge first
- [ ] STM32 wiring: USART3 TX/RX (PB10/PB11) connected to Pi GPIO14/15
- [ ] RC receiver bound to RadioMaster Boxer, signal cable on USART2 (PA3 RX)
- [ ] Camera USB cable seated on Pi OTG port
- [ ] Flight area clear, props-off bench test first if it's the first flight on a new build

---

## 2 · Power on STM32

1. Connect LiPo (or bench power via BEC at 5 V on the 5 V rail — **do not** power STM32 from USB only when ESCs are connected)
2. **Expected immediately:**
   - LED starts **slow 1 Hz blink** → Idle state (LD2 / PE1)
   - ESCs beep startup sequence (takes ~2 s while `motor_task` runs its startup delay)
3. **Keep the craft STILL for the first second.** The IMU runs a gyro-bias calibration from
   ~63 ms to ~663 ms after boot; motion during it trips the sanity check, the bias is
   discarded, and the whole session flies with zero bias compensation (one `warn!` line is
   the only indication).

   > ⚠ **The firmware's own prompt contradicts this step.** `mag_task` prints
   > **"CALIBRATION STARTING — ROTATE THE DRONE"** at ~35 ms, while the gyro calibration is
   > still running. **Ignore it for the first full second.** Rotating on cue corrupts the
   > gyro bias.

   If the IMU is missing or miswired, the LED switches to **rapid strobe** (Fault) and the
   task parks — check SPI1 wiring (PA4 CS, PA5 SCK, PA6 MISO, **PD7 MOSI**).
4. **Then rotate for the compass** — from ~1 s onward, slow figure-8 through all axes until
   the 25 s window closes (~25 s after boot). The window is min/max based with no outlier
   rejection, so keep motors off and phones away. Skip it and heading — plus PositionHold,
   Auto, RTH and FollowMe arming — stays unavailable until the next power cycle.
5. STM32 begins streaming MAVLink v2 on USART3 at 57600 baud — the Pi does not need to
   be ready yet; the heartbeat watchdog only engages after the *first* Pi heartbeat

---

## 3 · Start Pi services

The Pi services start automatically on boot if installed. After powering the Pi:

1. Wait ~30 s for the Pi Zero 2W to fully boot and connect to the hotspot
2. Verify both services are running:
   ```bash
   sudo systemctl status mavlink camera
   ```
   Both should show `Active: active (running)`.
3. If either is failed:
   ```bash
   sudo journalctl -u mavlink -n 30   # check for serial port errors, missing .env
   sudo journalctl -u camera -n 30    # check for camera device not found
   ```
4. Common issues:
   - `mavlink.service` fails → check `/dev/serial0` exists (`ls /dev/serial*`), UART not blocked by Bluetooth (`dtoverlay=disable-bt` in `/boot/firmware/config.txt`)
   - `camera.service` fails → check `/dev/video0` exists, Arducam USB cable seated
   - `LAPTOP_IP` wrong → GCS receives no telemetry; fix in `.env` and `sudo systemctl restart mavlink camera`

**What the Pi does once running:**
- Sends MAVLink HEARTBEAT to STM32 at 4 Hz → resets the 5-second watchdog
- Forwards STM32 telemetry bytes → GCS laptop on UDP:14550
- Forwards GCS commands (arm, mode change, mission) → STM32
- Streams H.264 camera feed → GCS on UDP:5600
- Listens for weed targets from GCS on UDP:5700

---

## 4 · Start Ground Station

On the laptop (connected to the same hotspot as the Pi):

```bash
cd ground
python gcs/station.py
```

The **pre-flight overlay opens automatically**. It shows a 7-item checklist and blocks arming until all items are green:

| Item | Passes when |
|---|---|
| **LINK** | MAVLink HEARTBEAT received from STM32 (Pi relay working) |
| **GPS** | 3D fix (fix_type ≥ 3, gnssFixOK flag set) — takes 30–90 s outdoors |
| **IMU** | SYS_STATUS sensor health bits for gyro + accel both set |
| **BARO** | SYS_STATUS sensor health bit for baro set |
| **BATTERY** | State of charge > 30 % |
| **THROTTLE** | RC collective throttle < 5 % |
| **HOME** | Home position locked. ⚠ **Only the SET HOME button sets this.** The firmware's automatic capture happens on the first navigation tick *after arming* (`navigation.rs:363` sits below the disarmed early-out), and it is never reported to the GCS. |

**If LINK never goes green:** the Pi relay is not forwarding telemetry. Check `mavlink.service` status and `LAPTOP_IP` in the Pi `.env`.

**Setting home manually:** if the HOME row shows a **SET HOME** button (GPS fix present), click it. This locks the current GPS position as the RTH origin and is required before arming.

---

## 5 · RC transmitter checks

> **Skip this section on the current build — no radio is fitted.** Arm via the GCS (6a).
> With no SBUS link seen, the RC gates are waived and the GCS disarm command is the only
> manual kill path.

With the RadioMaster Boxer powered on and bound:

- [ ] **Arm switch (Ch5) DOWN** — ⚠ **there is no power-on interlock.** `arming_task` is
      level-triggered, not edge-triggered: a switch left UP arms the craft as soon as
      `pre_arm_check` passes (~5 s after boot, gated by the battery hysteresis), and all four
      motors spin to the 4 % idle floor. Always power on with the switch DOWN.
- [ ] **Throttle stick at zero** (< 5%) — required by both RC and MAVLink arm paths
- [ ] **Mode switch (Ch6)** set to desired starting mode:
  - Position 1 (low) → **Stabilise** — manual, self-levelling only
  - Position 2 (mid) → **AltitudeHold** — baro altitude lock
  - Position 3 (high) → **Auto** (waypoint mission; holds position if no mission loaded)
- [ ] GCS THROTTLE row shows **ZERO** and LINK is green before proceeding

---

## 6 · Arming

There are two independent arm paths. Use **one** — not both simultaneously.

### 6a · Arm via GCS (recommended for autonomous missions)

1. Confirm all 7 pre-flight overlay items are green
2. Click the **ARM** button in the overlay — it pulses cyan while waiting for acknowledgement
3. GCS sends `MAVLink COMMAND_LONG (cmd=400, param1=1.0)` → Pi → STM32
4. STM32 `handle_command` checks, in order:
   - RC arm switch on, throttle < 5 %, no failsafe — **all three waived while no radio has
     been seen this boot** (`rc_gates_active()` is false)
   - Flight state is not Fault
   - **Full `pre_arm_check`** — IMU healthy · baro live (any mode except Stabilise) ·
     nav-ready GPS **and** calibrated compass (PositionHold / Auto / RTH / FollowMe) ·
     battery not critical
5. On acceptance: STM32 sets `armed = true`, state → **Armed**, sends `COMMAND_ACK`
6. GCS overlay receives `HEARTBEAT` with armed=true → button fades, overlay closes
7. LED switches to **double-pulse** pattern (Armed state)

> **Note:** the MAVLink path enforces the *same* `pre_arm_check` as the RC path, GPS and
> compass gates included — arming into Auto or PositionHold without a 3-D fix is rejected,
> not downgraded. What it waives (while no radio has been seen) are only the RC-derived
> gates: arm switch, throttle-low, and failsafe.

### 6b · Arm via RC (field / manual-only flights)

1. All checks from Section 5 complete. **The RBF pin is currently not enforced**
   (`SAFETY_PIN_ENABLED = false`) — pull it anyway as procedure, and re-enable the gate
   before field flights
2. Flip **arm switch (Ch5) UP**
3. `arming_task` checks every 20 ms:
   - RBF pin removed (PA0 open)
   - Arm switch high, throttle < 5 %, no failsafe, not in Fault state
   - ⚠ All four motors spin at idle (~4 %) the moment the craft arms — props clear!
   - IMU healthy (plausibility gates)
   - Any mode except Stabilise: **live barometer required**
   - PositionHold / Auto / RTH: **nav-ready GPS** (3-D fix, ≤ 5 m accuracy) **and calibrated compass**
4. LED switches to **double-pulse** (Armed)

### Arm rejected?

Each denial is logged with its reason (defmt/RTT). Common causes:

| Symptom | Cause |
|---|---|
| "remove-before-flight pin installed" | Pull the RBF jumper — *cannot occur while `SAFETY_PIN_ENABLED = false`* |
| GCS ARM button stays grey | Not all 7 checklist items green |
| GCS ARM pulses but never confirms | STM32 returned `MAV_RESULT_TEMPORARILY_REJECTED` — RC arm switch off, throttle not at zero, failsafe, Fault state, or a pre-arm gate (safety pin / IMU / baro / GPS / mag / battery critical) |
| "mode requires a live barometer" | Baro absent/dead and mode ≠ Stabilise |
| "mode requires nav-ready GPS" / "calibrated compass" | GPS-dependent mode without 3-D fix / mag figure-8 not done this boot |
| LED stays slow-blink after arm | Fault state — check defmt RTT log for cause |

---

## 7 · Takeoff and flight

1. **Spool up slowly** — throttle past 15% transitions state → **Flying** (LED solid on)
2. **First hover check (1 m AGL):** verify stability, confirm no oscillation on roll/pitch. If oscillating, land and reduce `rate_roll/pitch Kp` in `src/pid.rs`
3. **Mode switching:**
   - **Stabilise → AltitudeHold:** baro altitude locks at the moment you switch; throttle stick centres to hold
   - **AltitudeHold → PositionHold:** GPS position locks at the moment you switch
   - **Any mode → Auto:** mission runs only if waypoints were uploaded; otherwise holds current position
4. **Battery warnings:**
   - < 30% SoC → `navigation_task` forces RTH (GPS fix present) or Land (no fix)
   - < 3.30 V/cell under load (critical) → immediate forced RTH/Land; auto-disarms on ground; Fault state locked until battery swap
   - Note: arming is refused while the battery reads critical (also catches an uncalibrated V_DIVIDER reading 0 V)

---

## 8 · Landing and shutdown

### Normal landing
- **GCS:** click **LAND** in the program panel, or switch mode to Land (cmd 21)
- **RC:** ⚠ **Land is not on the mode switch.** Ch6 gives only Stabilise / AltitudeHold /
  Auto. The only RC landing path is switching to **Stabilise** and flying it down manually —
  and there is no throttle-position matching, so **pre-position the throttle stick to roughly
  hover (~55 %) before flipping the switch**, or the handover commands zero thrust

During landing:
- `navigation_task` descends at 0.5 m/s — a *descending target altitude* chased by the
  altitude PID, not an open-loop throttle reduction. Attitude is held level with **no
  position hold**, so it drifts downwind throughout the descent
- Control feedback is **always the barometer**. The MTF-02P rangefinder (below 2 m) is used
  only for **touchdown detection**, never for the descent itself
- Touchdown latches at **0.15 m AGL**, or after the descent target has sat at zero for 4 s
  (the backstop for a wrong baro datum). Throttle then cuts to zero and stays there

### Return to Home
- **GCS:** click **RETURN HOME** in program panel (sends cmd 20)
- **RC:** ⚠ **RTH is not on the mode switch either** — GCS only
- Cruise altitude is `max(current alt, 15 m)` — it **never descends**, so an altitude-geofence
  breach is not resolved by RTH
- Climb and transit happen **simultaneously**, not sequentially: it starts moving toward home
  immediately rather than clearing obstacles first
- Within 5 m of home it switches to the same `Lander` descent as above

### After landing
1. **Disarm:** flip arm switch DOWN (RC) or wait for auto-disarm on touchdown
2. **Disconnect LiPo** — do not leave LiPo connected unattended
3. Stop Pi services if done for the day: `sudo systemctl stop mavlink camera`
4. Quit GCS: `q` key or close window
5. Check LiPo voltage per cell — storage charge (3.8 V/cell) if not flying within 24 h

---

## Quick-reference: LED states

| Pattern | State | Meaning |
|---|---|---|
| 1 Hz slow blink | Idle | Powered, not armed |
| 5 Hz fast blink | Arming | *Unreachable — `FlightState::Arming` is never set anywhere in the firmware* |
| Double-pulse | Armed | Armed, motors live, throttle at zero |
| Solid on | Flying | Throttle > 15%, in flight |
| Triple-pulse every 2 s | Landing | Descent in progress |
| Rapid strobe | Fault | Hardware fault, critical battery, or Pi lost — **do not arm** |

## Quick-reference: Emergency procedures

| Situation | Action |
|---|---|
| Uncommanded movement | Flip arm switch DOWN immediately |
| GCS loses link mid-flight | Drone continues on last mode. ⚠ **With no radio fitted there is no override** — and if the *Pi* is what died, the 5 s watchdog forces an uncommandable Land |
| Pi heartbeat lost > 5 s | STM32 auto-forces Land, auto-disarms on ground |
| Critical battery in air | STM32 auto-forces RTH (GPS) or Land (no GPS) |
| IMU fault on power-on | LED strobes, `imu_task` parks; power off, check SPI1 wiring (**MOSI PD7**) |
| ESCs don't beep on power-on | ESC power side (battery/connector) — beeps are battery-driven, not signal-driven |
| ESC beeps but won't spin when armed | ESC never saw valid signal: check DShot ground continuity + 300 kbit rate (this ESC does not parse DShot600) |
