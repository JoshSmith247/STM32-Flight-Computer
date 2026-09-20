# Flight Computer — Study List

Prioritised reading order for interview prep. ~13 h total.
Codebase: 4,650 lines Rust, 16 concurrent tasks, STM32H723 / Embassy / `no_std`.

**Status:** ✅ walked through · ⬜ not yet

---

## WHERE YOU ARE  (updated 2026-09-19)

**Done thoroughly:** `main.rs` · `motor.rs` · `imu.rs` · `rc.rs` · `gps.rs` · `baro.rs` ·
**`navigation.rs` (complete)** · **`estimator.rs` (complete, line-level — see below)** ·
**`mag.rs` (2026-09-19, deep — see below)**

**In progress:** `navigation.rs` re-walked further 2026-09-19 (Q&A-driven, not systematic) —
still not fully closed out; original ~60% tracking below may be stale but the gaps
(PositionHold body, Auto body, FollowMe/RTH/DemoHover) haven't been confirmed closed.

### Plan for 2026-09-20
First-time/systematic reads: `flow.rs`, `baro.rs`.
Consolidation/review passes (already ✅ or absorbed via Q&A, going back through
end-to-end so it's one object instead of scattered facts): `rc.rs`, `battery.rs`,
`pid.rs`, `state.rs`, `types.rs`, `motor.rs`, `estimator.rs`, `health.rs`, `ahrs.rs`.

**Absorbed through questions, no systematic read needed:** `pid.rs` · `types.rs` ·
`health.rs` · `ahrs.rs`.

**`estimator.rs` — now genuinely deep, not just absorbed:** predict/correct/coast cycle,
why `origin_set` gates flow (it doubles as the `valid` flag — flow has no absolute
reference of its own, refuses to correct before a real GPS anchor exists), what
`DR_VEL_DECAY` actually does (decays the *state itself*, not a confidence/covariance
value — this struct has no uncertainty tracking at all, which is the structural
difference from an EKF), why flow needs both height (angle→body velocity) and yaw
(body→NED rotation), and how GPS velocity is even derived (Doppler shift per-satellite,
solved like the position fix but with frequency instead of pseudorange — and your
firmware just parses `vel_n/e/d_ms` out of UBX-PVT, it doesn't compute this itself).
Found a real naming bug here: `vel_x_mrad_s` claims milliradians but is actually
microradians (confirmed via `types.rs` comment, the `FLOW_SCALE` calibration comment,
and the `/1_000_000.0` in every consumer) — not a functional bug, everything used it
consistently, but a misleading identifier. Added to findings below.

### Bonus, not originally scoped: Embassy executor/async internals — done, deep
2026-09-04 second session went well past "conceptual" into verified source: the
run-queue is a lock-free intrusive stack (`cordyceps::TransferStack`, push-to-head,
drains in reverse-of-insertion order per batch); task state is 2 bits
(`STATE_SPAWNED`/`STATE_RUN_QUEUED`) packed in one atomic; DMA completions and
`Ticker`/`Timer` wakes are hardware-driven (ISR calls `wake()`) while `Mutex` wakes
are pure software (`MutexGuard::drop` calls it directly, no interrupt); the top-level
executor loop is `loop { poll(); wfe(); }` with no explicit empty-queue check, relying
on ARM's WFE/SEV event-latch to avoid the check-then-sleep race; `Mutex`'s single-slot
`WakerRegistration` handles multiple waiters by waking the displaced one on every
`register()`, producing a bounce/churn among contenders — never a deadlock, but no
fairness guarantee either. Also traced DMAMUX topology (DMAMUX1 shared by DMA1+DMA2,
16 channels; DMAMUX2 dedicated to unused BDMA) and confirmed via `Cargo.toml` this
project has zero `scheduler-priority`/`scheduler-deadline` — genuinely flat, no task
prioritization. Good general-purpose interview material beyond just this codebase.

### Actually remaining — ~0.8 h

- ✅ **`src/sensors/mag.rs`** (250) — done 2026-09-19: hard/soft-iron cal (per-axis
  min/max → offset + diagonal scale), why the 25 s rotate-through-all-orientations
  window, why cal failure blocks GPS-mode arming, tilt-compensated heading derivation
  (`bx/by/bz` + roll/pitch → `atan2` → `[0, 2π)`), the two-phase I2C read (write-then-read
  STATUS for DRDY, then write-then-read DATA), `with_timeout` guarding a stall-prone bus.
- ✅ **`src/sensors/battery.rs`** (124) — self-reported done.
- ⬜ **`src/pid.rs`** (226) — 30 min consolidation pass. You know every piece; read it
  end-to-end once so it's one object rather than six facts.
- ⬜ **`src/telemetry.rs`** (889) — 45 min skim. TX scheduling/staggering and the mission
  handshake are the parts you haven't seen. Skip the payload builders.
- ✅ **`src/state.rs`** (39) — self-reported done.
- ✅ **`src/sensors/mod.rs`** (22) — `read_exact_ring`, walked line-by-line 2026-09-04:
  fills a caller-sized buffer across multiple ring-buffer reads (not "drain everything"),
  why one-shot reads desync UBX/MICOLINK frame parsing, shared by `gps.rs` + `flow.rs`.

### Optional, depends on the team
- ⬜ `ground/gcs/` and `pi/` — ~1 h. Matters if Starlink puts you near ground software.

### NOT remaining — deliberately skipped
Madgwick's gradient-descent derivation · MS5611 second-order compensation · PLL internals ·
`servo.rs` / `led.rs` · SPI prescaler rounding

---

## TIER 1 — know cold. The interview lives here. (~7 h)

### ✅ `src/main.rs` (385)
- Boot sequence: clocks → watchdog → peripherals → spawn 16 tasks
- `control_task` (127–200) — the 12-step loop: snapshot, filter gyro, pick setpoint,
  fuse attitude, crash check, safety gate, anti-windup, PID cascade, mix, publish
- `pre_arm_check` (64) — the single gate both arm paths must pass
- `normal_arming` (208+) — arm/disarm, why Fault is never armable out of
- Panic handler — motors off, BSRR write, why `volatile`, why not ODR
- ⚠ `SAFETY_PIN_ENABLED = false` (53) — the RBF bypass

### 🔶 `src/navigation.rs` (965) — **in progress, ~60%**
Resume bullet #2 lives here. 100 Hz mode machine.
- ✅ Five-phase loop skeleton: gather → bookkeeping → decide → mode-entry → execute
- ✅ The `!is_armed` early-out (355) and why the tilt cutoff can't be landed out of
- ✅ Mode resolution chain (422–461): `mode` → `base_mode` → `effective_mode`
- ✅ Failsafe triggers (389–418), latching, and why geofence latches but GPS loss doesn't
- ✅ `guide_to` — flat-earth NED conversion, 20 m error clamp, yaw tracking above 3 m
- ✅ `Lander` — descending *target* not descent rate; two touchdown detections; `touched` latch
- ✅ Land vs RC-failsafe: controlled descent vs. direct motor zeroing
- ⬜ PositionHold body (529–587) — velocity damping, flow vs estimator fallback
- ⬜ Auto body (589–797) — mission sequencer, auto-takeoff, weed phases
- ⬜ FollowMe (799–837), RTH (839–864), DemoHover (877–948)

### ⬜ `src/health.rs` (97) — 30 min
- IMU plausibility bands, baro spike + liveness, GPS hacc bound, mag valid
- The comment saying these are **pre-arm gates, not in-flight failsafes**
- Why the spike gate adopts the bad value on the next tick

### ⬜ `src/types.rs` (367) — mostly declarations, 45 min
- `SharedState` — 20 separate mutexes; why not one lock over a snapshot
- The `is_fresh()` / `usable()` pattern and the comment explaining why
- Pessimistic defaults: battery `critical = true`, `hacc_m = 9999`

### ✅ `src/actuators/motor.rs` (~313)
- DShot frame: 11 throttle + 1 telemetry + 4 CRC; 48–2047 encoding
- ARR/CCR → bit period and duty cycle; 300 kbit/s vs 500 Hz frame rate
- `dshot_init` — timer config, burst DMA, DMAMUX request ID 27
- `dshot_send` — `m0ar` / `ndtr` / enable = source, count, go
- The spin-wait, why it exists, why it's the cadence-bug suspect
- **Best debugging story:** three stacked bugs

---

## TIER 2 — know the shape and the reasoning (~4 h)

### ⬜ `src/pid.rs` (226)
P / I / D, the cascade, `i_limit` sizing, derivative-on-measurement,
the stale-measurement hold, `mix_quad_x` offset trick, `HOVER = 0.55`

### ✅ `src/sensors/imu.rs` (220)
The driver pattern every other sensor follows: shared bus mutex,
datasheet delays, register config, burst read, freshness stamp

### ✅ `src/sensors/rc.rs` (214)
- SBUS bit unpacking — 16 × 11-bit channels from 22 bytes (also C++ bitmanip practice)
- The **four** failsafe paths and why each threshold differs: receiver flag (1 frame) ·
  `frame_lost` (250 ms) · 3 bad reads (~40 ms) · silence (100 ms)
- `frame_lost` = observation, `failsafe` = the receiver's conclusion
- The four consumers: motor cut (main.rs:189) · disarm (232) · arm denial (252) · GCS arm (telemetry:311)
- ⚠ Receiver configured to "hold last values" defeats every detection path — set outside the firmware
- Rust syntax picked up here: nested `Result`, `match` exhaustiveness, const-vs-binding in
  patterns, `saturating_add`, `get_or_insert_with`

### 🔶 `src/ahrs.rs` (147) — concept done, math deferred (by choice)
Fusion concept and why it exists is solid (complementary = high-pass fast-but-drifting
gyro + low-pass noisy-but-stable accel/mag, responses summing to unity — that's the name).
Still owe yourself the exact gradient-descent math, deliberately, another day —
this was never meant to be memorized, just not left as a total black box.

### ✅ `src/estimator.rs` (240)
Why complementary and not an EKF (no covariance/uncertainty tracked at all — `DR_VEL_DECAY`
decays the raw state, not a confidence value); the attitude → position dependency direction
(`predict()` rotates accel by the quaternion *first* — bad attitude corrupts position silently);
`origin_set` as the real meaning of "valid"; flow needs height (angle→velocity) *and* yaw
(body→NED); GPS velocity comes from per-satellite Doppler, not position differencing, and
this firmware just parses it out of UBX-PVT rather than computing it

### ✅ `src/state.rs` (39) — self-reported done
`AtomicU8`, six flight states, why atomics not a mutex

---

## TIER 3 — know what it does, don't memorise (~2 h)

- ⬜ `src/telemetry.rs` (889) — big but repetitive. Only the interesting ~150 lines:
  Pi heartbeat watchdog · MAVLink arm path (cmd 400) · in-air disarm magic `21196` ·
  mission upload handshake · hand-rolled CRC_EXTRA and byte offsets
- ⬜ `src/sensors/baro.rs` (169) — ground reference is **launch-relative, not MSL**
- ✅ `src/sensors/gps.rs` (180) — UBX sync-char frame finder, checksum, `parse_pvt` scaling;
  what a "fix" is (3 sats = 2D, 4 = 3D — clock offset is the 4th unknown); `hacc_m`;
  GPS as a *smart* sensor vs the IMU as a dumb one; ⚠ the no-timeout blocking bug
- ✅ `src/sensors/mag.rs` (250) — 25 s boot calibration, why failure blocks GPS modes
- ✅ `src/sensors/battery.rs` (124) — two-stage thresholds, asymmetric hysteresis;
  also its flow-as-rangefinder AGL check for the ground Fault-lock (found via the
  estimator sessions, not a standalone read)
- ⬜ `src/sensors/flow.rs` (116) — optional sensor, safe-when-absent pattern (the
  velocity/height math is already known cold from `estimator.rs`; this is just the
  MICOLINK frame-parsing file itself)
- ✅ `src/sensors/mod.rs` (22) — `read_exact_ring`; why one-shot reads drop bytes

---

## TIER 4 — skim once (~30 min)

- ⬜ `Cargo.toml` — pinned Embassy rev and why; no features left after the cleanup
- ⬜ `memory.x` — RAM at AXI SRAM `0x2400_0000`; DMA can't see DTCM
- ⬜ `.cargo/config.toml` — probe-rs runner, `DEFMT_LOG`
- ⬜ `src/actuators/payloads/servo.rs`, `src/status/led.rs` — trivial

---

## Not firmware, but on the resume

- ⬜ `ground/gcs/` — Python ground station, DearPyGui
- ⬜ `pi/` — MAVLink bridge, camera stream, weed pilot

---

## Quick reference — bus map

| Device | Bus | Pins | DMA | Rate |
|---|---|---|---|---|
| **IMU** ICM-42688-P | **SPI1** *shared* | SCK PA5 · MOSI PD7 · MISO PA6 · **CS PA4** | DMA2_CH3 tx / CH0 rx | 500 Hz |
| **Baro** MS5611 | **SPI1** *shared* | same bus · **CS PA8** | same | 25 Hz |
| **Mag** QMC5883L | **I2C1** | SCL PB8 · SDA PB9 | none — blocking | 25 Hz |
| **GPS** u-blox M10 | **USART1** | RX PA10 · TX PA9 | DMA2_CH6 tx / DMA2_CH5 rx | 115200 · ~5 Hz |
| **RC** SBUS | **USART2** | RX PA3 *(TX PA2 unused)* | DMA1_CH5 rx | 100k 8E2 inverted · ~71 Hz |
| **Telemetry** MAVLink | **USART3** | RX PB11 · TX PB10 | DMA1_CH3 tx / CH1 rx | 57600 · 10 Hz |
| **Flow** MTF-02P | **UART4** | RX PC11 *(rx only)* | DMA1_CH2 rx | 115200 · ~50 Hz |
| **Battery** | **ADC3** | PC0 volts · PF3 current | none — blocking | 2 Hz |
| **Motors** DShot300 | **TIM3** burst DMA | PB4 PB5 PB0 PB1 | DMA1_CH4 *hand-configured* | 500 Hz |
| **Servos** | **TIM4** PWM | PD12–PD15 | none | 50 Hz |
| Safety pin · grip · LED | GPIO | PA0 · PC2 · PE1 | — | — |

**Talking points:**
- Only **one shared bus** — SPI1, two devices, mutex-arbitrated with separate CS lines
- **Four UARTs, four protocols, four baud rates** — hence four frame parsers and the shared `read_exact_ring` helper
- **Two blocking drivers** — mag (I2C) and battery (ADC); everything else is async/DMA
- **9 of 16 DMA streams used.** `DMA1_CH4` is claimed by raw register writes in `dshot_init`, so the
  type system doesn't know it's taken — that's the "reserved, don't allocate elsewhere" comment
  in `motor.rs`, a comment doing a compiler's job

---

## Quick reference — clocks and rates

### MCU clock tree (`main.rs:407-436`)

```
HSI  64 MHz  (internal RC, ±1%, no crystal fitted)
 └─ PLL1  ÷4 → 16 MHz ref  ×50 → 800 MHz VCO
      ├─ ÷2 (divp) → 400 MHz  SYSCLK
      │    └─ ÷2 (AHB)  → 200 MHz  HCLK — CPU, memory, DMA
      │         ├─ ÷2 (APB1) → 100 MHz → TIM3/TIM4 kernel = 200 MHz (×2 rule)
      │         ├─ ÷2 (APB2) → 100 MHz
      │         └─ ÷2 (APB3/4) → 100 MHz
      └─ ÷8 (divq) → 100 MHz → SPI1, FDCAN

LSI  ~32 kHz (separate oscillator) → IWDG only — survives a PLL failure
```

### Bus signalling rates

| Bus | Rate | Note |
|---|---|---|
| SPI1 | requested 1 MHz → **actual ~781 kHz** | prescaler is a power of two; 100 MHz ÷ 128. *Verify against the HAL.* Was 12.5 MHz before the bus-loading diagnostic |
| I2C1 | 400 kHz | fast mode |
| USART1 GPS | 115200 | SEQURE M10-18 vendor default |
| USART2 SBUS | 100 000 | 8E2, **inverted** |
| USART3 MAVLink | 57600 | |
| UART4 flow | 115200 | MICOLINK |

### Timers

| Timer | Config | Result |
|---|---|---|
| **TIM3** DShot | kernel 200 MHz · PSC=1 → 100 MHz · ARR=332 | 333 ticks/bit → **300.3 kbit/s** (DShot300); 3.33 µs bit |
| **TIM4** servo | 50 Hz PWM | 1000–2000 µs pulses |
| **TIM5** | Embassy time driver | free 32-bit timer, app uses TIM3/TIM4 |
| **IWDG** | LSI ÷256, reload 250 | ~2.0 s nominal (LSI tolerance is wide) |

### Sensor internal rate vs. read rate — note the decimation everywhere

| Sensor | Internal | Task reads at | Discarded |
|---|---|---|---|
| IMU | 1 kHz ODR | 500 Hz | every 2nd sample ⚠ no anti-alias filter on that step |
| Mag | 100 Hz continuous | 25 Hz | 3 of every 4 |
| Baro | ~60 Hz max (8.22 ms × 2 conversions) | 25 Hz | — rate-limited by conversion time |
| GPS | 5 Hz nav rate | event-driven | — |
| Flow | ~50 Hz | event-driven | — |

### Task rates

| Rate | Tasks |
|---|---|
| 500 Hz | `control` · `imu` · `motor` |
| 150 Hz | `estimator` |
| 100 Hz | `navigation` |
| 50 Hz | `arming` · `servo` |
| 25 Hz | `baro` · `mag` |
| 20 Hz | `health` |
| 10 Hz | `telemetry` TX |
| event-driven | `rc` (≤100 ms) · `gps` (**no timeout**) · `flow` (≤500 ms) |
| 2 Hz | `battery` |

**Talking points:**
- 500 Hz is the *requested* rate — cadence clumping is an open bug, so it's a design target, not a measured fact
- Every filter and PID hardcodes `dt`; if the real interval isn't 2 ms the integrators are wrong and nothing detects it
- The IWDG runs off **LSI, a separate oscillator**, deliberately — so a PLL failure can't stop the thing that resets you out of it
- SPI at 781 kHz is a leftover diagnostic setting, not a designed value

---

## Concepts to be able to explain without notes

- Cooperative executor vs. RTOS — what you gained, what you gave up
- The data path: sensor → `STATE` → fusion → PID → mixer → DShot → ESC
- Why DMA exists (the bus is ~500× slower than the CPU)
- SPI vs UART failure modes when a device is unplugged
- The failure tree — three layers: reset / motor cut / mode degradation
- Integral windup and the three defences in this codebase
- Why the watchdog only proves one task is alive, and the fix


---

## Findings I made myself (interview material)

These came out of reading the code, not from anyone pointing at them.

1. **`Lander` can latch touchdown at altitude.** The baro fallback for touchdown detection
   (`navigation.rs:236-251`) uses a datum captured at boot that drifts ~8 m per hPa. With no
   usable rangefinder and a drifted baro, `agl < 0.15` can fire mid-air → `touched` latches →
   throttle 0, permanently. Affects *every* failsafe landing.
   *Fix:* require the commanded target to have reached zero too, or plausibility-check the baro.

2. **Telemetry reports the requested mode, not the flown mode.** `telemetry.rs:563` calls
   `STATE.effective_mode()` (RC/MAVLink resolution) while navigation computes a *different*
   local `effective_mode` after failsafes and demotions. Comment at :562 claims otherwise.
   GPS loss → demoted to AltitudeHold → GCS still shows PositionHold. Operator can't see it.
   *Fix:* publish navigation's resolved mode to `SharedState`; ~15 lines.

3. **`rc_gates_active()` means "a radio was fitted", not "a pilot is available".** Used at
   `navigation.rs:440` to decide Stabilise-vs-Land on altitude loss. It's a one-way boot latch
   (`RC_EVER_SEEN`), so it can't distinguish "no radio" from "radio died mid-flight". Behaviour
   is still safe because `control_task:189` cuts motors upstream — but the invariant is enforced
   in a different file from the one whose comment claims it.
   *Fix:* test `!rc.failsafe` at that decision point.

4. **No hysteresis on `baro_ok`.** `health.rs` header says in-flight checks "would want
   hysteresis" and then doesn't add any. A marginal baro flapping mid-descent oscillates
   `effective_mode` between Stabilise and AltitudeHold, resetting every PID on each flip
   (`navigation.rs:465-466`).

5. **Firmware's automatic home capture is invisible to the GCS.** `navigation.rs:363` sets home
   silently on the first fix; the GCS only knows about home if the operator pressed SET HOME.
   The two systems can hold different home positions with nothing detecting it.

6. **`estimator.rs`'s vertical channel is fused from GPS altitude, not baro — and nothing
   reads its output anyway.** `pos_d`/`vel_d` are corrected via `gps.alt_m`/`gps.vel_d_ms`
   (`correct_gps`); baro never appears in `estimator.rs` at all. Real altitude control lives
   entirely in `navigation.rs` off `baro.altitude_m` directly (":345, "Effective altitude
   comes from the barometer"), completely separate from the estimator. Checked every
   consumer of `STATE.pos_estimate` project-wide: the only external read is
   `navigation.rs:562`, and it only touches `vel_n`/`vel_e` — `pos_d`/`vel_d` are computed
   every tick and read by nobody. Backwards from convention too: baro is normally preferred
   for altitude hold on multirotors specifically because GPS vertical accuracy is much worse
   than horizontal (poor vertical DoP) — here the one vertical estimate trusting GPS altitude
   is also the one nothing consumes.

7. **`vel_x_mrad_s` / `vel_y_mrad_s` are misnamed — the values are microradians, not milli.**
   `types.rs:223-224` comments correctly say `urad/s`; `flow.rs:22`'s `FLOW_SCALE = 10_000`
   comment (`cm/s @ 1m -> urad/s`) confirms it; every consumer (`estimator.rs:225`,
   `navigation.rs:555`) divides by `1_000_000.0`, which is only correct for microradians.
   Not a functional bug — every use site agrees with the comment, not the name — but a
   misleading identifier that would burn anyone who trusted the field name over the code.
