# Startup Checklist

End-to-end procedure from power-off to armed and flying.  
Steps are ordered; each section depends on the previous one completing successfully.

---

## 0 · One-time setup (do once per build, not every flight)

These are prerequisites — if already done, skip to Section 1.

- [ ] **Flash firmware** — `cargo run --release`. On the **Nucleo** build with `--features nucleo`
      (Pi wiring) or `--features nucleo-vcp` (bench, MAVLink over ST-Link USB); probe-rs
      boots it directly. On the **custom FC** (DFU): true power-cycle after every flash —
      CR3 is write-once per POR and a soft reset hangs the time driver (board looks dead)
- [ ] **Wire the safety inputs** — remove-before-flight jumper header on **PA0↔GND**;
      gripper-jaw microswitch on **PC2↔GND** (weed missions, `--features grip-sense`)
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
   - LED starts **slow 1 Hz blink** → Idle state (PG7 on the FC; LD2 on the Nucleo)
   - ESCs beep startup sequence (takes ~2 s while `motor_task` runs its startup delay)
3. **Keep the craft STILL for the first second** — the IMU runs a gyro-bias calibration
   (~0.6 s) at boot; motion during it degrades attitude for the whole session. If the IMU
   fails (wiring fault) the flight build LED switches to **rapid strobe** (Fault) — check
   SPI1 wiring (PA4 CS, PA5 SCK, PA6 MISO, MOSI = PA7 on FC / **PD7 on Nucleo**). Bench
   `nucleo-vcp` builds warn and continue instead of faulting
4. **If the compass is fitted:** rotate the craft in a slow figure-8 through all axes for
   the 25 s calibration window, or heading (and GPS-mode arming) stays unavailable
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
| **HOME** | Home position locked (set manually via **SET HOME** button, or auto-captured on first 3D fix) |

**If LINK never goes green:** the Pi relay is not forwarding telemetry. Check `mavlink.service` status and `LAPTOP_IP` in the Pi `.env`.

**Setting home manually:** if the HOME row shows a **SET HOME** button (GPS fix present), click it. This locks the current GPS position as the RTH origin and is required before arming.

---

## 5 · RC transmitter checks

With the RadioMaster Boxer powered on and bound:

- [ ] **Arm switch (Ch5) DOWN** — throttle interlock; drone rejects arm if switch is up at power-on
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
4. STM32 `handle_command` checks:
   - RC throttle < 5 %
   - RC link not in failsafe
   - Flight state is not Fault
5. On acceptance: STM32 sets `armed = true`, state → **Armed**, sends `COMMAND_ACK`
6. GCS overlay receives `HEARTBEAT` with armed=true → button fades, overlay closes
7. LED switches to **double-pulse** pattern (Armed state)

> **Note:** MAVLink arm does not enforce a GPS fix check even in GPS-dependent modes. If you arm via GCS in Auto or PositionHold without a 3D fix, `navigation_task` will warn and fall back to AltitudeHold.

### 6b · Arm via RC (field / manual-only flights)

1. All checks from Section 5 complete, **and the remove-before-flight pin is PULLED**
2. Flip **arm switch (Ch5) UP**
3. `arming_task` checks every 20 ms:
   - RBF pin removed (PA0 open)
   - Arm switch high, throttle < 5 %, no failsafe, not in Fault state
   - IMU healthy (plausibility gates)
   - Any mode except Stabilise: **live barometer required**
   - PositionHold / Auto / RTH: **nav-ready GPS** (3-D fix, ≤ 5 m accuracy) **and calibrated compass**
4. LED switches to **double-pulse** (Armed)

### Arm rejected?

Each denial is logged with its reason (defmt/RTT). Common causes:

| Symptom | Cause |
|---|---|
| "remove-before-flight pin installed" | Pull the RBF jumper |
| GCS ARM button stays grey | Not all 7 checklist items green |
| GCS ARM pulses but never confirms | STM32 returned `MAV_RESULT_UNSUPPORTED` — throttle not at zero, failsafe active, or Fault state |
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
   - < 20% SoC → `navigation_task` forces RTH (GPS fix present) or Land (no fix)
   - < 3.5 V/cell → immediate forced Land; auto-disarms on ground; Fault state locked until battery swap

---

## 8 · Landing and shutdown

### Normal landing
- **RC:** switch to Land mode (Ch6 position 3 or flip arm switch down to cut throttle)
- **GCS:** click **LAND** in the program panel, or switch mode to Land

During landing:
- `navigation_task` descends at 0.5 m/s
- Below 2 m AGL the MTF-02P rangefinder takes over from baro
- Below 5 cm AGL throttle cuts to zero, armed = false, state → Idle

### Return to Home
- **GCS:** click **RETURN HOME** in program panel (sends cmd 20)
- **RC:** switch mode to RTH position
- Drone climbs to max(current alt, 15 m), flies home on haversine bearing, then descends

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
| 5 Hz fast blink | Arming | Arm switch held, transitioning |
| Double-pulse | Armed | Armed, motors live, throttle at zero |
| Solid on | Flying | Throttle > 15%, in flight |
| Triple-pulse every 2 s | Landing | Descent in progress |
| Rapid strobe | Fault | Hardware fault, critical battery, or Pi lost — **do not arm** |

## Quick-reference: Emergency procedures

| Situation | Action |
|---|---|
| Uncommanded movement | Flip arm switch DOWN immediately |
| GCS loses link mid-flight | Drone continues on last mode; RC has full override |
| Pi heartbeat lost > 5 s | STM32 auto-forces Land, auto-disarms on ground |
| Critical battery in air | STM32 auto-forces RTH (GPS) or Land (no GPS) |
| IMU fault on power-on | LED strobes; power off, check SPI1 wiring (MOSI: PA7 FC / PD7 Nucleo) |
| ESCs don't beep on power-on | ESC power side (battery/connector) — beeps are battery-driven, not signal-driven |
| ESC beeps but won't spin when armed | ESC never saw valid signal: check DShot ground continuity + 300 kbit rate (this ESC does not parse DShot600) |
