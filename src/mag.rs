//! QMC5883L magnetometer driver — I2C1 blocking, 25 Hz output.
//!
//! I2C address: 0x0D
//! Pinout: PB8 = SCL, PB9 = SDA (I2C1 AF4)
//!
//! Init sequence:
//!   1. Soft reset (CR2 = 0x80), wait 10 ms
//!   2. Set/Reset Period register = 0x01 (required by datasheet)
//!   3. CR2 = 0x00 (clear)
//!   4. CR1 = 0x19 — continuous mode, 100 Hz ODR, 8 G range, OSR=512
//!
//! Each 25 Hz tick: poll DRDY (status bit 0), read 6 raw bytes, apply
//! tilt compensation using the current AHRS quaternion, write STATE.mag_data.

use core::f32::consts::TAU;

use defmt::{error, info, warn};
use embassy_stm32::{
    i2c::{Config, I2c},
    peripherals,
    time::hz,
    Peri,
};
use embassy_time::{Duration, Ticker, Timer};

use crate::{types::MagData, STATE};

const ADDR: u8 = 0x0D;

const REG_DATA:    u8 = 0x00; // X_L, X_H, Y_L, Y_H, Z_L, Z_H
const REG_STATUS:  u8 = 0x06;
const REG_CR1:     u8 = 0x09;
const REG_CR2:     u8 = 0x0A;
const REG_PERIOD:  u8 = 0x0B;
const REG_CHIP_ID: u8 = 0x0D;

// CR1: OSR=512 | RNG=8G | ODR=100Hz | Mode=Continuous
// bits [7:6]=00  [5:4]=01  [3:2]=10  [1:0]=01  → 0b00_01_10_01 = 0x19
const CR1_VALUE: u8 = 0x19;

// ── Hard-iron calibration offsets ─────────────────────────────────────────────
// With the drone fully assembled and all electronics powered, rotate it slowly
// in a figure-8 pattern. Record the min and max raw ADC count on each axis and
// set each offset to (max + min) / 2.  All zeros = uncalibrated.
const MAG_OFFSET_X: f32 = 0.0;
const MAG_OFFSET_Y: f32 = 0.0;
const MAG_OFFSET_Z: f32 = 0.0;

fn tilt_compensated_heading(bx: f32, by: f32, bz: f32, roll: f32, pitch: f32) -> f32 {
    let (sr, cr) = (libm::sinf(roll),  libm::cosf(roll));
    let (sp, cp) = (libm::sinf(pitch), libm::cosf(pitch));
    let xh = bx * cp + by * sr * sp - bz * cr * sp;
    let yh = by * cr + bz * sr;
    let mut heading = libm::atan2f(-yh, xh);
    if heading < 0.0 { heading += TAU; }
    heading
}

#[embassy_executor::task]
pub async fn mag_task(
    i2c: Peri<'static, peripherals::I2C1>,
    scl: Peri<'static, peripherals::PB8>,
    sda: Peri<'static, peripherals::PB9>,
) {
    Timer::after(Duration::from_millis(20)).await;

    let mut cfg = Config::default();
    cfg.frequency = hz(400_000);
    cfg.scl_pullup = true;
    cfg.sda_pullup = true;
    let mut dev = I2c::new_blocking(i2c, scl, sda, cfg);

    // Soft reset
    if dev.blocking_write(ADDR, &[REG_CR2, 0x80]).is_err() {
        error!("Mag: I2C write failed on reset — check wiring (continuing without mag)");
        loop { Timer::after(Duration::from_secs(10)).await; }
    }
    Timer::after(Duration::from_millis(10)).await;

    dev.blocking_write(ADDR, &[REG_PERIOD, 0x01]).ok();
    dev.blocking_write(ADDR, &[REG_CR2, 0x00]).ok();
    dev.blocking_write(ADDR, &[REG_CR1, CR1_VALUE]).ok();

    let mut chip_id = [0u8; 1];
    dev.blocking_write_read(ADDR, &[REG_CHIP_ID], &mut chip_id).ok();
    if chip_id[0] != 0xFF {
        warn!("Mag: unexpected chip ID 0x{:02X} (expected 0xFF)", chip_id[0]);
    }

    info!("Mag: QMC5883L ready — 100 Hz continuous, 8 G range");

    let mut ticker = Ticker::every(Duration::from_hz(25));

    loop {
        ticker.next().await;

        let mut status = [0u8; 1];
        if dev.blocking_write_read(ADDR, &[REG_STATUS], &mut status).is_err() {
            warn!("Mag: status read failed");
            continue;
        }
        if status[0] & 0x01 == 0 { continue; } // DRDY not set

        let mut raw = [0u8; 6];
        if dev.blocking_write_read(ADDR, &[REG_DATA], &mut raw).is_err() {
            warn!("Mag: data read failed");
            continue;
        }

        let x = i16::from_le_bytes([raw[0], raw[1]]) as f32 - MAG_OFFSET_X;
        let y = i16::from_le_bytes([raw[2], raw[3]]) as f32 - MAG_OFFSET_Y;
        let z = i16::from_le_bytes([raw[4], raw[5]]) as f32 - MAG_OFFSET_Z;

        let q = *STATE.attitude.lock().await;
        let roll  = libm::atan2f(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        );
        let pitch = libm::asinf((2.0 * (q.w * q.y - q.z * q.x)).clamp(-1.0, 1.0));

        let heading_rad = tilt_compensated_heading(x, y, z, roll, pitch);

        *STATE.mag_data.lock().await = MagData { x, y, z, heading_rad, valid: true };
    }
}
