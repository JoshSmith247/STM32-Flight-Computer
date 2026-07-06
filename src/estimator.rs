//! Complementary NED position/velocity estimator (deliberately not an EKF):
//! trusts IMU dead-reckoning on short timescales, pulls toward GPS on long,
//! optional low-altitude flow correction. Publishes `STATE.pos_estimate`.
//!
//! Output frame: NED (x=N, y=E, z=Down). Origin = first valid GPS 3-D fix
//! (same point navigation captures as home). Flat-earth conversion:
//!   meters_n = dlat_deg * 111_320;  meters_e = dlon_deg * 111_320 * cos(lat)
//!
//! Runs at 150 Hz.

use defmt::info;
use embassy_time::{Duration, Instant, Ticker};
use libm::cosf;

use crate::{
    types::{PosEstimate, Quaternion, Vec3},
    STATE,
};

// ── Tuning ──────────────────────────────────────────────────────────────────

const RATE_HZ: u64 = 150;
const DT: f32 = 1.0 / RATE_HZ as f32;
const GRAVITY: f32 = 9.81; // m/s² on the NED Down axis

/// Complementary correction gains (per GPS update). These are applied as a
/// fraction of the residual between the dead-reckoned estimate and the GPS
/// measurement each time a fresh fix arrives. Small position gain (slow trust
/// in absolute GPS position), larger velocity gain (GPS NED velocity is far
/// less noisy than differentiated position on the M8N).
///
/// Effective complementary time constant τ ≈ dt_gps / gain. At ~5 Hz GPS:
///   pos: τ ≈ 0.2 / 0.10 ≈ 2 s   vel: τ ≈ 0.2 / 0.30 ≈ 0.7 s
const K_POS_GPS: f32 = 0.10;
const K_VEL_GPS: f32 = 0.30;

/// Optical-flow horizontal velocity correction gain, used only at low altitude
/// where the rangefinder/flow is reliable. Weak — flow is a complement to,
/// not a replacement for, GPS velocity.
const K_VEL_FLOW: f32 = 0.05;
const FLOW_MAX_HEIGHT_MM: i32 = 5_000;

/// If accel-only dead-reckoning persists this long without any GPS correction,
/// bleed velocity toward zero so a stuck/biased accel doesn't run the position
/// estimate away unbounded during a long dropout.
const DR_VEL_DECAY: f32 = 0.999; // per-tick multiplier when coasting

// ── Filter state ──────────────────────────────────────────────────────────────

/// Complementary NED position/velocity estimator.
pub struct PosEstimator {
    // NED position (m, relative to origin) and velocity (m/s).
    pos_n: f32,
    pos_e: f32,
    pos_d: f32,
    vel_n: f32,
    vel_e: f32,
    vel_d: f32,

    // NED origin (geodetic). Captured from the first valid GPS fix.
    origin_lat_deg: f64,
    origin_lon_deg: f64,
    origin_alt_m: f32,
    origin_set: bool,
}

impl PosEstimator {
    pub const fn new() -> Self {
        Self {
            pos_n: 0.0, pos_e: 0.0, pos_d: 0.0,
            vel_n: 0.0, vel_e: 0.0, vel_d: 0.0,
            origin_lat_deg: 0.0, origin_lon_deg: 0.0, origin_alt_m: 0.0,
            origin_set: false,
        }
    }

    /// Convert a geodetic position to local NED metres relative to the origin.
    /// Same flat-earth approximation navigation.rs uses for short ranges.
    fn geo_to_ned(&self, lat_deg: f64, lon_deg: f64, alt_m: f32) -> (f32, f32, f32) {
        let lat_rad = (self.origin_lat_deg as f32).to_radians();
        let n = ((lat_deg - self.origin_lat_deg) * 111_320.0) as f32;
        let e = ((lon_deg - self.origin_lon_deg) * 111_320.0 * cosf(lat_rad) as f64) as f32;
        // Down is positive: descending below origin altitude increases pos_d.
        let d = self.origin_alt_m - alt_m;
        (n, e, d)
    }

    /// High-rate PREDICT: rotate body specific force into the world frame,
    /// convert NWU→NED, remove gravity, integrate to velocity then position.
    fn predict(&mut self, accel_body: Vec3, q: Quaternion) {
        // R(q) rotates body → Madgwick world frame = Z-UP / NWU
        // (X=North, Y=West, Z=Up — see ahrs::ned_yaw).
        let (w, x, y, z) = (q.w, q.x, q.y, q.z);

        let r00 = 1.0 - 2.0 * (y * y + z * z);
        let r01 = 2.0 * (x * y - w * z);
        let r02 = 2.0 * (x * z + w * y);
        let r10 = 2.0 * (x * y + w * z);
        let r11 = 1.0 - 2.0 * (x * x + z * z);
        let r12 = 2.0 * (y * z - w * x);
        let r20 = 2.0 * (x * z - w * y);
        let r21 = 2.0 * (y * z + w * x);
        let r22 = 1.0 - 2.0 * (x * x + y * y);

        // Specific force in the NWU world frame.
        let f_n  = r00 * accel_body.x + r01 * accel_body.y + r02 * accel_body.z;
        let f_w  = r10 * accel_body.x + r11 * accel_body.y + r12 * accel_body.z;
        let f_up = r20 * accel_body.x + r21 * accel_body.y + r22 * accel_body.z;

        // NWU → NED: N = X, E = -Y, D = -Z. At rest f_up = +g, so linear
        // down-acceleration a_d = g - f_up = 0.
        // ⚠ BRING-UP: lift the board sharply and confirm vel_d goes negative
        // (up) before trusting the vertical channel.
        let an = f_n;
        let ae = -f_w;
        let ad = GRAVITY - f_up;

        // Integrate accel → velocity → position (forward Euler).
        self.vel_n += an * DT;
        self.vel_e += ae * DT;
        self.vel_d += ad * DT;

        self.pos_n += self.vel_n * DT;
        self.pos_e += self.vel_e * DT;
        self.pos_d += self.vel_d * DT;
    }

    /// Low-rate CORRECT step from a fresh GPS fix. Blends the dead-reckoned
    /// estimate toward GPS NED position and GPS NED velocity.
    fn correct_gps(
        &mut self,
        lat_deg: f64, lon_deg: f64, alt_m: f32,
        vel_n: f32, vel_e: f32, vel_d: f32,
    ) {
        if !self.origin_set {
            self.origin_lat_deg = lat_deg;
            self.origin_lon_deg = lon_deg;
            self.origin_alt_m = alt_m;
            self.origin_set = true;
            // Seed the estimate at the origin with GPS velocity.
            self.pos_n = 0.0; self.pos_e = 0.0; self.pos_d = 0.0;
            self.vel_n = vel_n; self.vel_e = vel_e; self.vel_d = vel_d;
            info!("Estimator origin set: lat={=f64} lon={=f64}", lat_deg, lon_deg);
            return;
        }

        let (gps_n, gps_e, gps_d) = self.geo_to_ned(lat_deg, lon_deg, alt_m);

        // Position correction toward GPS.
        self.pos_n += K_POS_GPS * (gps_n - self.pos_n);
        self.pos_e += K_POS_GPS * (gps_e - self.pos_e);
        self.pos_d += K_POS_GPS * (gps_d - self.pos_d);

        // Velocity correction toward GPS NED velocity.
        self.vel_n += K_VEL_GPS * (vel_n - self.vel_n);
        self.vel_e += K_VEL_GPS * (vel_e - self.vel_e);
        self.vel_d += K_VEL_GPS * (vel_d - self.vel_d);
    }

    /// Optional low-altitude optical-flow horizontal velocity correction.
    /// `flow_vel_n/e` are flow-derived velocities already rotated into NED.
    fn correct_flow(&mut self, flow_vel_n: f32, flow_vel_e: f32) {
        self.vel_n += K_VEL_FLOW * (flow_vel_n - self.vel_n);
        self.vel_e += K_VEL_FLOW * (flow_vel_e - self.vel_e);
    }

    fn snapshot(&self) -> PosEstimate {
        PosEstimate {
            pos_n: self.pos_n, pos_e: self.pos_e, pos_d: self.pos_d,
            vel_n: self.vel_n, vel_e: self.vel_e, vel_d: self.vel_d,
            valid: self.origin_set,
        }
    }
}

// ── Embassy task ──────────────────────────────────────────────────────────────

#[embassy_executor::task]
pub async fn estimator_task() {
    let mut est = PosEstimator::new();

    // GPS arrives much slower than the predict loop; only run a correction when
    // a genuinely new fix is seen. The M8N fix doesn't carry a timestamp here,
    // so we detect change via lat/lon deltas plus a minimum interval.
    let mut last_gps_lat: f64 = 0.0;
    let mut last_gps_lon: f64 = 0.0;
    let mut last_correct: Instant = Instant::now();
    let mut had_recent_gps = false;

    info!("Estimator task started ({=u64} Hz)", RATE_HZ);
    let mut ticker = Ticker::every(Duration::from_hz(RATE_HZ));

    loop {
        ticker.next().await;

        // ── PREDICT from IMU ────────────────────────────────────────────────
        let imu = *STATE.imu_data.lock().await;
        let q = *STATE.attitude.lock().await;
        est.predict(imu.accel, q);

        // ── CORRECT from GPS (fresh valid fix only) ──────────────────────────
        let gps = *STATE.gps_fix.lock().await;
        if gps.usable() {
            let moved = (gps.lat_deg != last_gps_lat) || (gps.lon_deg != last_gps_lon);
            // Correct on a new fix, or at least every 250 ms to keep tracking
            // even when stationary (lat/lon static but velocity meaningful).
            if moved || last_correct.elapsed() > Duration::from_millis(250) {
                est.correct_gps(
                    gps.lat_deg, gps.lon_deg, gps.alt_m,
                    gps.vel_n_ms, gps.vel_e_ms, gps.vel_d_ms,
                );
                last_gps_lat = gps.lat_deg;
                last_gps_lon = gps.lon_deg;
                last_correct = Instant::now();
                had_recent_gps = true;
            }
        } else if last_correct.elapsed() > Duration::from_secs(2) {
            // Long GPS dropout: coast on dead-reckoning but bleed velocity so a
            // biased accel can't diverge unbounded.
            est.vel_n *= DR_VEL_DECAY;
            est.vel_e *= DR_VEL_DECAY;
            est.vel_d *= DR_VEL_DECAY;
            had_recent_gps = false;
        }

        // ── Optional CORRECT from optical flow at low altitude ───────────────
        // Only fused when we lack a recent GPS update (flow then carries the
        // horizontal velocity estimate during dropout near the ground).
        if est.origin_set && !had_recent_gps {
            let flow = *STATE.flow.lock().await;
            if flow.usable() && flow.height_mm > 0 && flow.height_mm <= FLOW_MAX_HEIGHT_MM {
                let h_m = flow.height_mm as f32 / 1000.0;
                // Body-frame velocities from flow (matches navigation.rs scaling).
                let vel_fwd = flow.vel_x_mrad_s as f32 * h_m / 1_000_000.0;
                let vel_right = flow.vel_y_mrad_s as f32 * h_m / 1_000_000.0;

                // Rotate body horizontal velocity into NED using the NED yaw.
                let yaw = crate::ahrs::ned_yaw(&q);
                let (cy, sy) = (cosf(yaw), libm::sinf(yaw));
                let flow_vel_n = vel_fwd * cy - vel_right * sy;
                let flow_vel_e = vel_fwd * sy + vel_right * cy;
                est.correct_flow(flow_vel_n, flow_vel_e);
            }
        }

        // ── Publish ──────────────────────────────────────────────────────────
        *STATE.pos_estimate.lock().await = est.snapshot();
    }
}
