//! ICM-42688-P IMU driver - SPI1 async DMA (bus shared with the baro), CS=PA4.
//! ODR 1 kHz, task reads at 500 Hz so every read is fresh data.

use core::f32::consts::PI;

use defmt::{error, info, warn}; // warn used by gyro-cal (all builds) + nucleo-vcp IMU-missing path
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    peripherals,
    Peri,
};
use embassy_time::{Duration, Ticker, Timer};

#[cfg(not(feature = "nucleo-vcp"))] // only the flight build Faults on a missing IMU
use crate::state::FlightState;
use crate::{
    types::{ImuData, Vec3},
    STATE,
};

// Register map

const WHO_AM_I:      u8 = 0x75;
const DEVICE_CONFIG: u8 = 0x11;
const PWR_MGMT0:     u8 = 0x4E;
const GYRO_CONFIG0:  u8 = 0x4F;
const ACCEL_CONFIG0: u8 = 0x50;
const TEMP_DATA1:    u8 = 0x1D; // first byte of 14-byte sensor block

const WHO_AM_I_EXPECTED: u8 = 0x47;

// Scale factors (datasheet 3.1 / 3.2)

// full-scale 2000 dps -> rad/s per LSB
const GYRO_SCALE:  f32 = (2000.0 / 32768.0) * (PI / 180.0);
// full-scale 16 g -> m/s^2 per LSB
const ACCEL_SCALE: f32 = (16.0 / 32768.0) * 9.80665;

// Startup gyro-bias calibration

// Stationary samples averaged for the bias estimate (~0.6 s at 500 Hz).
const CAL_SAMPLES: u32 = 300;

// Averaged per-axis gyro above this means the board was moving during cal:
// far above stationary MEMS bias, well below any real hand motion.
const CAL_SANITY_RAD_S: f32 = 0.05;

// SPI transaction helpers

/// Write a register: assert CS, clock out [addr, val], deassert CS.
async fn write_reg(cs: &mut Output<'_>, addr: u8, val: u8) {
    let mut buf = [addr & 0x7F, val];
    let mut bus = crate::SPI1_BUS.lock().await;
    let Some(spi) = bus.as_mut() else { return; };
    cs.set_low();
    spi.transfer_in_place(&mut buf).await.ok();
    cs.set_high();
}

/// Read one register: addr | 0x80 to signal a read, returns the data byte.
async fn read_reg(cs: &mut Output<'_>, addr: u8) -> u8 {
    let mut buf = [addr | 0x80, 0x00];
    let mut bus = crate::SPI1_BUS.lock().await;
    let Some(spi) = bus.as_mut() else { return 0; };
    cs.set_low();
    spi.transfer_in_place(&mut buf).await.ok();
    cs.set_high();
    buf[1]
}

/// Burst-read the 14-byte sensor block and return the gyro XYZ in rad/s.
/// Returns `None` if the SPI bus is unavailable or the transfer faults.
async fn read_gyro(cs: &mut Output<'_>) -> Option<Vec3> {
    let mut buf = [0u8; 15];
    buf[0] = TEMP_DATA1 | 0x80;
    {
        let mut bus = crate::SPI1_BUS.lock().await;
        let spi = bus.as_mut()?;
        cs.set_low();
        if spi.transfer_in_place(&mut buf).await.is_err() {
            cs.set_high();
            return None;
        }
        cs.set_high();
    }
    Some(Vec3 {
        x: i16::from_be_bytes([buf[9],  buf[10]]) as f32 * GYRO_SCALE,
        y: i16::from_be_bytes([buf[11], buf[12]]) as f32 * GYRO_SCALE,
        z: i16::from_be_bytes([buf[13], buf[14]]) as f32 * GYRO_SCALE,
    })
}

/// Average CAL_SAMPLES stationary gyro samples into a bias vector. An implausibly
/// large result (board moved) is discarded for a zero bias, with a warn.
async fn calibrate_gyro_bias(cs: &mut Output<'_>, ticker: &mut Ticker) -> Vec3 {
    info!("IMU: calibrating gyro bias — keep the board still ({} samples)", CAL_SAMPLES);

    let (mut sx, mut sy, mut sz) = (0.0f32, 0.0f32, 0.0f32);
    let mut n: u32 = 0;
    while n < CAL_SAMPLES {
        ticker.next().await;
        if let Some(g) = read_gyro(cs).await {
            sx += g.x;
            sy += g.y;
            sz += g.z;
            n += 1;
        }
    }

    let inv = 1.0 / n as f32;
    let bias = Vec3 { x: sx * inv, y: sy * inv, z: sz * inv };

    // Per-axis magnitude check (no sqrt needed in no_std hot path).
    let moved = bias.x.abs() > CAL_SANITY_RAD_S
        || bias.y.abs() > CAL_SANITY_RAD_S
        || bias.z.abs() > CAL_SANITY_RAD_S;
    if moved {
        warn!(
            "IMU: gyro-cal bias implausibly large (x={} y={} z={} rad/s) — board moved? \
             discarding, using zero bias",
            bias.x, bias.y, bias.z
        );
        return Vec3 { x: 0.0, y: 0.0, z: 0.0 };
    }

    info!("IMU: gyro bias = (x={} y={} z={}) rad/s", bias.x, bias.y, bias.z);
    bias
}

// Embassy task

#[embassy_executor::task]
pub async fn imu_task(cs_pin: Peri<'static, peripherals::PA4>) {
    let mut cs = Output::new(cs_pin, Level::High, Speed::High);

    // 1 ms minimum from VDD stable to first SPI transaction (datasheet 6.1).
    Timer::after(Duration::from_millis(10)).await;

    // Soft reset - clears all registers to default state.
    write_reg(&mut cs, DEVICE_CONFIG, 0x01).await;
    // Device needs up to 1 ms to complete reset before accepting new commands.
    Timer::after(Duration::from_millis(2)).await;

    // Confirm we're talking to the right chip. Retried: one flaky SPI read at
    // power-up must not permanently Fault the flight build.
    #[cfg_attr(feature = "simulation", allow(unused_assignments, unused_mut))]
    let mut who = read_reg(&mut cs, WHO_AM_I).await;
    for attempt in 1..3u8 {
        if who == WHO_AM_I_EXPECTED { break; }
        warn!("IMU: WHO_AM_I = 0x{:02X} (attempt {}/3) — retrying", who, attempt);
        Timer::after(Duration::from_millis(10)).await;
        who = read_reg(&mut cs, WHO_AM_I).await;
    }
    #[cfg(feature = "simulation")]
    let who = WHO_AM_I_EXPECTED;
    if who != WHO_AM_I_EXPECTED {
        // Flight build: a missing IMU is fatal - Fault and park.
        #[cfg(not(feature = "nucleo-vcp"))]
        {
            error!("ICM-42688-P not found: WHO_AM_I = 0x{:02X} (expected 0x{:02X})", who, WHO_AM_I_EXPECTED);
            crate::state::set(FlightState::Fault);
            loop { Timer::after(Duration::from_secs(1)).await; }
        }
        // Bench build: tolerate a missing IMU so motor tests can run. WARNING: DO NOT FLY.
        #[cfg(feature = "nucleo-vcp")]
        {
            warn!("ICM-42688-P not found (WHO_AM_I=0x{:02X}) — bench build, continuing WITHOUT IMU (no Fault, DO NOT FLY)", who);
            loop { Timer::after(Duration::from_secs(1)).await; }
        }
    }
    info!("IMU: ICM-42688-P found (WHO_AM_I = 0x{:02X})", who);

    // PWR_MGMT0: gyro low-noise (bits 3:2 = 11), accel low-noise (bits 1:0 = 11).
    write_reg(&mut cs, PWR_MGMT0, 0x0F).await;
    Timer::after(Duration::from_millis(1)).await;

    // GYRO_CONFIG0: FS = +/-2000 dps (bits 7:5 = 000), ODR = 1 kHz (bits 3:0 = 0110).
    write_reg(&mut cs, GYRO_CONFIG0, 0x06).await;

    // ACCEL_CONFIG0: FS = +/-16g (bits 7:5 = 000), ODR = 1 kHz (bits 3:0 = 0110).
    write_reg(&mut cs, ACCEL_CONFIG0, 0x06).await;

    // Gyro startup in low-noise mode takes up to 45 ms (datasheet Table 1).
    Timer::after(Duration::from_millis(50)).await;

    let mut ticker = Ticker::every(Duration::from_hz(500));

    // Startup gyro-bias calibration - board must be still (checked inside).
    let gyro_bias = calibrate_gyro_bias(&mut cs, &mut ticker).await;

    info!("IMU: running — ±2000 dps / ±16g @ 1 kHz ODR, sampling at 500 Hz");

    loop {
        ticker.next().await;

        // Burst-read 14 bytes: 2 temp + 6 accel + 6 gyro (regs 0x1D-0x2A).
        let mut buf = [0u8; 15];
        buf[0] = TEMP_DATA1 | 0x80;
        {
            let mut bus = crate::SPI1_BUS.lock().await;
            let Some(spi) = bus.as_mut() else { continue; };
            cs.set_low();
            if spi.transfer_in_place(&mut buf).await.is_err() {
                cs.set_high();
                error!("IMU: SPI transfer error");
                continue;
            }
            cs.set_high();
        }

        // Temperature ( degC) - datasheet 4.17: Temp_degC = raw / 132.48 + 25
        let temp_raw = i16::from_be_bytes([buf[1], buf[2]]);
        let temp_c   = (temp_raw as f32 / 132.48) + 25.0;

        // Accelerometer XYZ (m/s²)
        let ax = i16::from_be_bytes([buf[3],  buf[4]])  as f32 * ACCEL_SCALE;
        let ay = i16::from_be_bytes([buf[5],  buf[6]])  as f32 * ACCEL_SCALE;
        let az = i16::from_be_bytes([buf[7],  buf[8]])  as f32 * ACCEL_SCALE;

        // Gyroscope XYZ (rad/s), with the startup bias removed so attitude
        // integration doesn't drift from a stationary offset.
        let gx = i16::from_be_bytes([buf[9],  buf[10]]) as f32 * GYRO_SCALE - gyro_bias.x;
        let gy = i16::from_be_bytes([buf[11], buf[12]]) as f32 * GYRO_SCALE - gyro_bias.y;
        let gz = i16::from_be_bytes([buf[13], buf[14]]) as f32 * GYRO_SCALE - gyro_bias.z;

        *STATE.imu_data.lock().await = ImuData {
            accel: Vec3 { x: ax, y: ay, z: az },
            gyro:  Vec3 { x: gx, y: gy, z: gz },
            temp_c,
            stamp_ms: crate::types::stamp_now_ms(),
        };
    }
}
