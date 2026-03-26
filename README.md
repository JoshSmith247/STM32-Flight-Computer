# STM32 Flight Computer

Autonomous quadrotor flight computer written in Rust, targeting the **STM32F405RGT6** (Cortex-M4F, 168 MHz).

## Architecture

```
src/
├── main.rs         — System init, clock config, task spawning, 500 Hz control loop
├── types.rs        — Shared data types (Vec3, Quaternion, RcInput, MotorOutputs …)
├── imu.rs          — ICM-42688-P driver (SPI1, 24 MHz) — 2000 Hz task
├── ahrs.rs         — Madgwick AHRS filter → quaternion attitude estimate
├── pid.rs          — Two-stage PID cascade (attitude → rate) + quad-X motor mixer
├── motor.rs        — DSHOT600 ESC output (TIM3 DMA) — 500 Hz task
├── rc.rs           — SBUS RC receiver parser (USART2, 100 kbaud) — IRQ-driven task
├── baro.rs         — MS5611 barometer driver (SPI1) — 50 Hz task
├── navigation.rs   — Waypoint sequencer, RTH, land, position-hold — 100 Hz task
└── telemetry.rs    — MAVLink v2 GCS stream + command receive (USART3) — 100 Hz task
```

## Hardware

| Component    | Part            | Interface       |
|--------------|-----------------|-----------------|
| MCU          | STM32F405RGT6   | —               |
| IMU          | ICM-42688-P     | SPI1 @ 24 MHz   |
| Barometer    | MS5611          | SPI1 (shared)   |
| GPS          | u-blox M8N      | USART1 @ 9600   |
| RC receiver  | SBUS            | USART2 @ 100 k  |
| ESCs (×4)    | DSHOT600        | TIM3 CH1–4      |
| Telemetry    | MAVLink v2      | USART3 @ 57600  |

## Task Timing

| Task           | Rate    | Notes                          |
|----------------|---------|--------------------------------|
| `imu_task`     | 2000 Hz | IMU read + Madgwick filter     |
| `control_task` | 500 Hz  | PID cascade + motor mixer      |
| `baro_task`    | 50 Hz   | Pressure → altitude            |
| `navigation_task` | 100 Hz | Waypoint sequencer          |
| `telemetry_task` | 100 Hz | MAVLink heartbeat + streams  |
| `rc_task`      | IRQ     | SBUS frame parser              |
| `motor_task`   | 500 Hz  | DSHOT600 DMA output            |

## Building

Install the Rust embedded toolchain:

```sh
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools
```

Build and flash (requires a debug probe — ST-Link or J-Link):

```sh
cargo build --release
cargo run --release     # flashes via probe-rs and opens RTT log
```

## Flight Modes

| Mode          | Description                              |
|---------------|------------------------------------------|
| Stabilise     | Self-levelling, manual throttle/yaw      |
| AltitudeHold  | Barometer-assisted altitude hold         |
| PositionHold  | GPS position hold                        |
| Auto          | Follow uploaded waypoint mission         |
| ReturnToHome  | Fly back to arming position and land     |
| Land          | Descend at 0.5 m/s until disarm          |

## Status

This is an initial code outline. Items marked `TODO` in source require hardware bring-up:

- DSHOT600 DMA buffer implementation (`motor.rs`)
- SPI bus sharing between IMU and barometer (`baro.rs`)
- GPS NMEA/UBX parser (`navigation.rs`)
- Full MAVLink ATTITUDE / VFR_HUD / GPS_RAW_INT frames (`telemetry.rs`)
- Full Madgwick Jacobian (`ahrs.rs`)
- PID gain tuning on real hardware (`pid.rs`)
