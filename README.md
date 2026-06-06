# STM32 Flight Computer

Autonomous quadrotor flight computer written in Rust, targeting the **STM32H723ZGT6** (Cortex-M7F, 400 MHz).

## Architecture

```
src/
├── main.rs          — System init, clock config (400 MHz PLL), task spawning, hardware watchdog
├── types.rs         — Shared data types (Vec3, Quaternion, RcInput, MotorOutputs …)
├── imu.rs           — ICM-42688-P driver (SPI1, 12.5 MHz) — 500 Hz task
├── mag.rs           — QMC5883L compass driver (I2C1, 400 kHz) — 25 Hz task
├── baro.rs          — MS5611 barometer driver (SPI1 shared) — 25 Hz task
├── gps.rs           — u-blox M10 UBX NAV-PVT parser (USART1) — 1 Hz task
├── flow.rs          — MTF-02P optical flow + rangefinder (UART4) — frame-driven task
├── battery.rs       — LiPo voltage monitor via ADC3 (PC0) — 2 Hz task
├── ahrs.rs          — Madgwick 9-DOF AHRS filter → quaternion attitude estimate
├── pid.rs           — PID cascade (angle → rate → torque) + quad-X motor mixer
├── motor.rs         — DSHOT600 ESC output (TIM3 burst DMA) — 500 Hz task
├── rc.rs            — SBUS RC receiver parser (USART2, 100 kbaud, 8E2) — IRQ-driven task
├── navigation.rs    — Waypoint sequencer, RTH, land, position-hold, weed targeting — 100 Hz task
├── telemetry.rs     — MAVLink v2 GCS stream + command receive (USART3, 57600) — 10 Hz task
├── state.rs         — Flight state machine (Idle/Arming/Armed/Flying/Landing/Fault)
├── led.rs           — Status LED blink patterns (6 states)
└── payloads/
    └── servo.rs     — 4-channel PWM servo output (TIM4, 50 Hz)
```

**Ground station & Pi relay:**
```
ground/gcs/          — PyDearGUI GCS (telemetry HUD, MAVLink mission builder, weed map, YOLOv8)
pi/                  — Raspberry Pi Zero 2W relay scripts (MAVLink bridge, weed targeting, camera stream)
```

## Hardware

| Component       | Part               | Interface              |
|-----------------|--------------------|------------------------|
| MCU             | STM32H723ZGT6      | —                      |
| IMU             | ICM-42688-P        | SPI1 @ 12.5 MHz        |
| Barometer       | MS5611             | SPI1 (shared mutex)    |
| Compass         | QMC5883L           | I2C1 @ 400 kHz         |
| GPS             | u-blox M10         | USART1 @ 38400         |
| Optical flow    | MTF-02P            | UART4 @ 19200          |
| RC receiver     | ELRS / SBUS        | USART2 @ 100 kbaud 8E2 |
| ESCs (×4)       | DSHOT600           | TIM3 CH1–4 (DMA)       |
| Servos (×4)     | PWM 50 Hz          | TIM4 CH1–4             |
| Battery monitor | ADC divider        | ADC3 PC0               |
| Telemetry       | MAVLink v2         | USART3 @ 57600         |
| Companion       | Raspberry Pi Zero 2W | USART3 ↔ UDP         |
| Camera          | Arducam OV9782     | USB UVC (on Pi)        |

## Task Timing

| Task              | Rate       | Notes                                   |
|-------------------|------------|-----------------------------------------|
| `control_task`    | 500 Hz     | Madgwick fusion + PID cascade + mixer   |
| `imu_task`        | 500 Hz     | SPI burst read (1 kHz ODR)              |
| `motor_task`      | 500 Hz     | DSHOT600 DMA encode + send              |
| `arming_task`     | 50 Hz      | Arm/disarm logic + RC watchdog          |
| `navigation_task` | 100 Hz     | Waypoint sequencer + guidance PIDs      |
| `telemetry_task`  | 10 Hz TX   | MAVLink v2 7-msg cycle + mission RX     |
| `baro_task`       | 25 Hz      | Pressure → altitude (ISA model)         |
| `mag_task`        | 25 Hz      | Tilt-compensated heading                |
| `servo_task`      | 50 Hz      | TIM4 PWM duty update                    |
| `battery_task`    | 2 Hz       | ADC voltage → %, critical failsafe      |
| `gps_task`        | 1 Hz       | UBX NAV-PVT parse                       |
| `flow_task`       | frame      | MTF-02P height + body-frame velocity    |
| `rc_task`         | IRQ        | SBUS frame parser (100 ms timeout)      |
| `led_task`        | 1–5 Hz     | Status LED blink pattern                |

## Flight Modes

| Mode           | Description                                              |
|----------------|----------------------------------------------------------|
| Stabilise      | Self-levelling, manual throttle/yaw                     |
| AltitudeHold   | Barometer altitude lock + RC roll/pitch/yaw             |
| PositionHold   | GPS + optical flow position lock                        |
| Auto           | Follow uploaded MAVLink waypoint mission                |
| ReturnToHome   | Climb to 15 m, fly home via haversine, descend          |
| Land           | Rate-controlled descent, rangefinder touch-down detect  |
| FollowMe       | Track GCS-sent position (YOLOv8 person detection)       |

## Building

Install the Rust embedded toolchain:

```sh
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools
```

Build and flash (requires ST-Link or J-Link debug probe):

```sh
cargo build --release
cargo run --release     # flashes via probe-rs and opens RTT log
```

## Ground Station

```sh
cd ground
pip install -r requirements.txt
python gcs/station.py
```

Requires the Pi relay to be running (`pi/weed_pilot.py`) and the drone on the same WiFi network as the laptop (default: Pi at `192.168.4.1`, laptop at `192.168.4.2`).

## Startup & Pre-flight

See **[STARTUP_CHECKLIST.md](STARTUP_CHECKLIST.md)** for the full end-to-end procedure — Pi setup, GCS startup, arming via RC or ground station, flight, and emergency procedures.

## One-time Hardware Checklist

- [ ] Compass hard-iron calibration (`MAG_OFFSET_X/Y/Z` in `mag.rs`)
- [ ] Verify voltage divider ratio matches `V_DIVIDER` in `battery.rs`
- [ ] Confirm DSHOT DMAMUX request ID against RM0468 Table 105 (`motor.rs`)
- [ ] ELRS receiver configured to SBUS output mode
- [ ] Power-cycle STM32 after each flash (CR3 write-once-per-POR)
