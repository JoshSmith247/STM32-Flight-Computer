//! Sensor health/plausibility monitor (~20 Hz): publishes per-sensor flags to
//! `STATE.sensor_health`, gating the pre-arm checks. Checks are deliberately
//! wide pre-arm gates, not in-flight failsafes (those would want hysteresis).

use defmt::{info, warn};
use embassy_time::{Duration, Instant, Ticker};
use libm::sqrtf;

use crate::{types::SensorHealth, STATE};

const RATE_HZ: u64 = 20;

// IMU plausibility: accel magnitude in a sane band around 1 g, gyro not pinned
// near full-scale.
const G: f32 = 9.80665;
const ACCEL_MIN: f32 = 0.25 * G; // ~2.45 m/s²
const ACCEL_MAX: f32 = 4.0 * G;  // ~39.2 m/s² - sustained >4 g is implausible at rest
const GYRO_SAT_RAD_S: f32 = 0.95 * (2000.0 * core::f32::consts::PI / 180.0); // 95 % of FS

// GPS "usable for nav" gate: a 3-D fix with horizontal accuracy within this bound.
const GPS_HACC_MAX_M: f32 = 5.0;

// Baro spike rejection: a single-tick altitude jump larger than this (at 20 Hz, i.e.
// 50 ms) implies hundreds of m/s and is non-physical - flag the baro for that sample.
const BARO_SPIKE_M: f32 = 20.0;

// Baro liveness: a real MS5611 jitters by centimetres every sample - no change
// within this window means the driver/bus died and the value is frozen.
const BARO_LIVE_WINDOW: Duration = Duration::from_secs(5);

#[embassy_executor::task]
pub async fn health_task() {
    let mut ticker = Ticker::every(Duration::from_hz(RATE_HZ));
    let mut last_baro_alt: Option<f32> = None;
    let mut last_baro_change: Option<Instant> = None;
    let mut prev = SensorHealth::default();

    info!("Health monitor started ({=u64} Hz)", RATE_HZ);

    loop {
        ticker.next().await;

        // IMU: fresh + accel magnitude sane + gyro not saturated + finite
        let imu = *STATE.imu_data.lock().await;
        let (a, g) = (imu.accel, imu.gyro);
        let amag = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        let imu_ok = imu.is_fresh()
            && amag.is_finite()
            && amag >= ACCEL_MIN && amag <= ACCEL_MAX
            && g.x.is_finite() && g.y.is_finite() && g.z.is_finite()
            && g.x.abs() < GYRO_SAT_RAD_S
            && g.y.abs() < GYRO_SAT_RAD_S
            && g.z.abs() < GYRO_SAT_RAD_S;

        // Baro: finite + no implausible single-tick spike + actually alive
        let alt = STATE.baro_data.lock().await.altitude_m;
        if let Some(prev_alt) = last_baro_alt {
            if alt.is_finite() && (alt - prev_alt).abs() > 0.001 {
                last_baro_change = Some(Instant::now());
            }
        }
        let baro_alive = last_baro_change.map_or(false, |t| t.elapsed() < BARO_LIVE_WINDOW);
        let baro_ok = alt.is_finite()
            && baro_alive
            && last_baro_alt.map_or(true, |prev_alt| (alt - prev_alt).abs() < BARO_SPIKE_M);
        if alt.is_finite() {
            last_baro_alt = Some(alt);
        }

        // Mag: calibration succeeded AND data recently written
        let mag_ok = STATE.mag_data.lock().await.usable();

        // GPS: fresh 3-D fix with bounded horizontal accuracy
        let gps = *STATE.gps_fix.lock().await;
        let gps_ok = gps.usable()
            && gps.hacc_m.is_finite()
            && gps.hacc_m < GPS_HACC_MAX_M;

        let health = SensorHealth { imu_ok, baro_ok, mag_ok, gps_ok };
        *STATE.sensor_health.lock().await = health;

        // Edge-triggered logging - announce only on health transitions, not every tick.
        if imu_ok != prev.imu_ok {
            if imu_ok { info!("Health: IMU ok"); } else { warn!("Health: IMU UNHEALTHY"); }
        }
        if baro_ok != prev.baro_ok {
            if baro_ok { info!("Health: baro ok"); } else { warn!("Health: baro UNHEALTHY"); }
        }
        if mag_ok != prev.mag_ok {
            if mag_ok { info!("Health: compass calibrated/ok"); } else { warn!("Health: compass not ready"); }
        }
        if gps_ok != prev.gps_ok {
            if gps_ok { info!("Health: GPS nav-ready"); } else { warn!("Health: GPS not nav-ready"); }
        }
        prev = health;
    }
}
