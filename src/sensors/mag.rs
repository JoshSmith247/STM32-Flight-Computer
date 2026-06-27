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

// ── Runtime magnetometer calibration ──────────────────────────────────────────
// We compute hard-iron + simple diagonal soft-iron calibration at every boot by
// watching the raw vector while the USER ROTATES THE DRONE through all
// orientations during a short window. There is NO flash persistence — the cal is
// thrown away on reset and MUST be redone every power cycle. This is intentional
// (out of scope); see the project notes.
//
// >>> THE USER MUST PHYSICALLY ROTATE THE DRONE (slow figure-8, hitting every
// >>> axis +/-) for the entire calibration window or heading will be garbage. <<<
//
// Hard-iron offset (per axis) = (min + max) / 2.
// Soft-iron is approximated as a diagonal scale that equalises the per-axis
// ranges: scale_axis = avg_range / range_axis. This corrects only axis-aligned
// scale distortion, not cross-axis (off-diagonal) terms — adequate for heading.

/// How long the rotate-the-drone calibration window lasts. The user must keep
/// rotating the airframe through all orientations for this whole duration.
const CAL_WINDOW_SECS: u64 = 25;

/// Set to `false` to skip calibration entirely (publish raw, no offset/scale).
/// Useful on the bench when no field rotation is possible.
const CAL_ENABLED: bool = true;

/// Per-axis range below this (in LSB) means the user almost certainly did NOT
/// rotate the drone; we then fall back to identity scale to avoid blowing up a
/// near-zero range into a huge scale factor.
const MIN_VALID_RANGE: f32 = 50.0;

/// Hard-iron offset (subtracted) and diagonal soft-iron scale (multiplied)
/// applied to every raw reading after the calibration window completes.
#[derive(Clone, Copy)]
struct MagCal {
    offset: [f32; 3],
    scale:  [f32; 3],
}

impl MagCal {
    /// Identity: no offset, unity scale (uncalibrated passthrough).
    const fn identity() -> Self {
        MagCal { offset: [0.0; 3], scale: [1.0; 3] }
    }

    #[inline]
    fn apply(&self, v: [f32; 3]) -> [f32; 3] {
        [
            (v[0] - self.offset[0]) * self.scale[0],
            (v[1] - self.offset[1]) * self.scale[1],
            (v[2] - self.offset[2]) * self.scale[2],
        ]
    }
}

/// Compute hard-iron offset + diagonal soft-iron scale from observed per-axis
/// min/max. Falls back to identity scale on any axis whose range is implausibly
/// small (drone was not actually rotated).
fn compute_cal(min: [f32; 3], max: [f32; 3]) -> MagCal {
    let mut offset = [0.0f32; 3];
    let mut range  = [0.0f32; 3];
    for i in 0..3 {
        offset[i] = (min[i] + max[i]) * 0.5;
        range[i]  = max[i] - min[i];
    }
    let avg_range = (range[0] + range[1] + range[2]) / 3.0;

    let mut scale = [1.0f32; 3];
    if avg_range >= MIN_VALID_RANGE {
        for i in 0..3 {
            scale[i] = if range[i] >= MIN_VALID_RANGE {
                avg_range / range[i]
            } else {
                1.0
            };
        }
    }
    MagCal { offset, scale }
}

/// Poll DRDY and read one raw magnetometer vector (LSB, sensor frame).
/// Returns `None` if data is not ready or an I2C read fails (caller skips tick).
fn read_raw(
    dev: &mut I2c<'static, embassy_stm32::mode::Blocking, embassy_stm32::i2c::Master>,
) -> Option<[f32; 3]> {
    let mut status = [0u8; 1];
    if dev.blocking_write_read(ADDR, &[REG_STATUS], &mut status).is_err() {
        warn!("Mag: status read failed");
        return None;
    }
    if status[0] & 0x01 == 0 { return None; } // DRDY not set

    let mut raw = [0u8; 6];
    if dev.blocking_write_read(ADDR, &[REG_DATA], &mut raw).is_err() {
        warn!("Mag: data read failed");
        return None;
    }

    Some([
        i16::from_le_bytes([raw[0], raw[1]]) as f32,
        i16::from_le_bytes([raw[2], raw[3]]) as f32,
        i16::from_le_bytes([raw[4], raw[5]]) as f32,
    ])
}

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

    // ── Calibration window ─────────────────────────────────────────────────────
    // NOTE: NOT persisted to flash — recalibrated on every boot, by design.
    let cal = if CAL_ENABLED {
        warn!(
            "Mag: CALIBRATION STARTING — ROTATE THE DRONE through ALL orientations \
             (slow figure-8) for the next {} s!",
            CAL_WINDOW_SECS
        );

        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        let mut samples: u32 = 0;
        let total_ticks = CAL_WINDOW_SECS * 25;
        // Log a progress line ~ once per second.
        let log_every = 25u64;

        for tick in 0..total_ticks {
            ticker.next().await;

            if let Some(v) = read_raw(&mut dev) {
                for i in 0..3 {
                    if v[i] < min[i] { min[i] = v[i]; }
                    if v[i] > max[i] { max[i] = v[i]; }
                }
                samples += 1;
            }

            if (tick + 1) % log_every == 0 {
                let secs_left = (total_ticks - tick - 1) / 25;
                info!(
                    "Mag cal: {} s left, {} samples — KEEP ROTATING — \
                     X[{=f32}..{=f32}] Y[{=f32}..{=f32}] Z[{=f32}..{=f32}]",
                    secs_left, samples,
                    min[0], max[0], min[1], max[1], min[2], max[2]
                );
            }
        }

        if samples == 0 || min[0].is_infinite() {
            warn!("Mag cal: no samples captured — publishing UNCALIBRATED (identity)");
            MagCal::identity()
        } else {
            let cal = compute_cal(min, max);
            info!(
                "Mag cal DONE ({} samples): offset[{=f32},{=f32},{=f32}] \
                 scale[{=f32},{=f32},{=f32}]",
                samples,
                cal.offset[0], cal.offset[1], cal.offset[2],
                cal.scale[0],  cal.scale[1],  cal.scale[2]
            );
            cal
        }
    } else {
        warn!("Mag: calibration DISABLED (CAL_ENABLED=false) — publishing raw readings");
        MagCal::identity()
    };

    loop {
        ticker.next().await;

        let raw = match read_raw(&mut dev) {
            Some(v) => v,
            None => continue,
        };

        let [x, y, z] = cal.apply(raw);

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
