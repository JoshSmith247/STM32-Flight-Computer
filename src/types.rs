//! Shared data types and inter-task state.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::Instant;

// ---------------------------------------------------------------------------
// Basic geometric types
// ---------------------------------------------------------------------------

/// Roll / pitch / yaw in radians.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct Euler {
    pub roll:  f32,
    pub pitch: f32,
    pub yaw:   f32,
}

/// Unit quaternion representing vehicle attitude.
#[derive(Clone, Copy, defmt::Format)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Default for Quaternion {
    fn default() -> Self { Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 } }
}

/// 3-axis vector (accel m/s², gyro rad/s, or NED position m).
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Geographic coordinate (WGS-84).
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct LatLonAlt {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m:   f32,
}

/// Live GPS fix from u-blox M8N, updated by gps_task.
#[derive(Clone, Copy, defmt::Format)]
pub struct GpsFix {
    pub lat_deg:  f64,
    pub lon_deg:  f64,
    pub alt_m:    f32,
    pub vel_n_ms: f32,    // NED velocity m/s
    pub vel_e_ms: f32,
    pub vel_d_ms: f32,
    pub hacc_m:   f32,    // horizontal accuracy estimate
    pub fix_ok:   bool,   // true = gnssFixOK flag set and fix_type >= 3
    pub fix_type: u8,     // NAV-PVT fixType: 0=none 1=DR 2=2D 3=3D 4=combined
}

impl Default for GpsFix {
    fn default() -> Self {
        Self { lat_deg: 0.0, lon_deg: 0.0, alt_m: 0.0,
               vel_n_ms: 0.0, vel_e_ms: 0.0, vel_d_ms: 0.0,
               hacc_m: 9999.0, fix_ok: false, fix_type: 0 }
    }
}

impl GpsFix {
    /// Extract a plain waypoint coordinate for use with navigation helpers.
    pub fn pos(&self) -> LatLonAlt {
        LatLonAlt { lat_deg: self.lat_deg, lon_deg: self.lon_deg, alt_m: self.alt_m }
    }
}

// ---------------------------------------------------------------------------
// Sensor data
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Default, defmt::Format)]
pub struct ImuData {
    pub accel: Vec3,        // m/s²
    pub gyro:  Vec3,        // rad/s
    pub temp_c: f32,
}

#[derive(Clone, Copy, Default, defmt::Format)]
pub struct BaroData {
    pub pressure_pa: f32,
    pub temp_c:      f32,
    pub altitude_m:  f32,   // derived via ISA model
}

/// Battery state — updated at 2 Hz by battery_task.
#[derive(Clone, Copy, defmt::Format)]
pub struct BatteryData {
    pub voltage_v: f32,
    pub pct:       u8,      // 0–100 %
    pub critical:  bool,    // true when per-cell V < 3.50 V for ≥ 2.5 s
    /// Measured pack current from the ESC's CUR pad (A). Negative = sensor not
    /// fitted/calibrated — consumers (telemetry) then fall back to the
    /// throttle-based estimate.
    pub current_a: f32,
}

impl Default for BatteryData {
    fn default() -> Self {
        // Pessimistic defaults: GCS shows 0 V / critical until battery_task
        // confirms healthy voltage (~2.5 s after boot). Prevents the operator
        // from receiving a false "battery OK" before the first ADC reading.
        Self { voltage_v: 0.0, pct: 0, critical: true, current_a: -1.0 }
    }
}

// ---------------------------------------------------------------------------
// RC / pilot input
// ---------------------------------------------------------------------------

/// Normalised RC channels in [-1.0, 1.0] (throttle 0..1).
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct RcInput {
    pub throttle: f32,      // 0.0 – 1.0
    pub roll:     f32,      // ±1.0
    pub pitch:    f32,      // ±1.0
    pub yaw:      f32,      // ±1.0
    pub arm:      bool,
    pub mode:     FlightMode,
    pub failsafe: bool,
}

impl RcInput {
    /// Convert raw stick positions to an attitude setpoint (radians / rad·s⁻¹).
    pub fn to_attitude_setpoint(&self) -> AttitudeSetpoint {
        const MAX_ANGLE: f32 = 0.5236; // 30 deg
        const MAX_RATE:  f32 = 3.1416; // 180 deg/s yaw
        AttitudeSetpoint {
            roll:     self.roll  * MAX_ANGLE,
            pitch:    self.pitch * MAX_ANGLE,
            yaw_rate: self.yaw   * MAX_RATE,
            throttle: self.throttle,
        }
    }
}

#[derive(Clone, Copy, Default, defmt::Format, PartialEq)]
pub enum FlightMode {
    #[default]
    Stabilise,
    AltitudeHold,
    PositionHold,
    Auto,           // Follow waypoint mission / weed picking
    ReturnToHome,
    Land,
    FollowMe,       // Continuous person tracking via GCS YOLOv8; discriminant = 6
}

// ---------------------------------------------------------------------------
// Control setpoints
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Default, defmt::Format)]
pub struct AttitudeSetpoint {
    pub roll:     f32,      // rad
    pub pitch:    f32,      // rad
    pub yaw_rate: f32,      // rad/s
    pub throttle: f32,      // 0.0 – 1.0
}

#[derive(Clone, Copy, Default, defmt::Format)]
pub struct NavCommand {
    pub autonomous:        bool,
    pub attitude_setpoint: AttitudeSetpoint,
    pub target:            LatLonAlt,
}

// ---------------------------------------------------------------------------
// Motor outputs (DSHOT / PWM normalised 0.0 – 1.0)
// ---------------------------------------------------------------------------

/// Motor mixer output for a quad in X configuration. Fields map to:
///   m1 front-right (CW)    m2 front-left (CCW)
///   m3 back-right  (CCW)   m4 back-left  (CW)
/// Diagonal pairs share rotation direction: {m1,m4} CW, {m2,m3} CCW.
/// Spin direction must still be verified on the bench before first spin-up.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct MotorOutputs {
    pub m1: f32,
    pub m2: f32,
    pub m3: f32,
    pub m4: f32,
}

/// Bench bring-up motor test, set by MAV_CMD_DO_MOTOR_TEST (209). Honoured ONLY
/// by motor_task while disarmed and on the ground (FlightState::Idle), and auto-
/// expired at `until` — so a stale override can never spin a motor in flight.
/// Use with PROPS OFF to verify per-corner wiring and spin direction.
#[derive(Clone, Copy)]
pub struct MotorTest {
    pub idx:      u8,       // 1..=4 → firmware M1..M4
    pub throttle: f32,      // 0.0..=1.0, clamped low at the command site
    pub until:    Instant,  // spin stops once Instant::now() >= until
}

// ---------------------------------------------------------------------------
// Global shared state (Embassy async mutexes, ISR-safe)
// ---------------------------------------------------------------------------

/// Optical flow + rangefinder data from MTF-02P, updated by flow_task.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct FlowData {
    pub quality:      u8,    // 0–255
    pub vel_x_mrad_s: i32,   // body-frame X velocity (mrad/s × 1000)
    pub vel_y_mrad_s: i32,   // body-frame Y velocity (mrad/s × 1000)
    pub height_mm:    i32,   // rangefinder reading in mm; -1 = no data
    pub valid:        bool,  // quality > 50 and recent frame received
}

/// Fused NED position/velocity estimate, updated by estimator_task (~150 Hz).
/// Position is metres relative to the NED origin (first valid GPS fix / home);
/// velocity is m/s in NED. `valid` becomes true once the origin is captured.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct PosEstimate {
    pub pos_n: f32,    // North position (m, relative to origin)
    pub pos_e: f32,    // East  position (m)
    pub pos_d: f32,    // Down  position (m, +down)
    pub vel_n: f32,    // North velocity (m/s)
    pub vel_e: f32,    // East  velocity (m/s)
    pub vel_d: f32,    // Down  velocity (m/s)
    pub valid: bool,   // true once the NED origin has been set from GPS
}

/// Per-sensor health/plausibility flags, updated by health_task (~20 Hz). Each flag is
/// true only when that sensor is producing physically plausible, usable data. Consumed
/// by the pre-arm checks in arming_task (and available for future failsafe logic).
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct SensorHealth {
    pub imu_ok:  bool,   // accel magnitude near 1 g + gyro not saturated + finite
    pub baro_ok: bool,   // altitude finite + no implausible single-tick spike
    pub mag_ok:  bool,   // calibrated/valid compass data
    pub gps_ok:  bool,   // 3-D fix with horizontal accuracy within bound
}

/// Magnetometer data from QMC5883L, updated by mag_task at 25 Hz.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct MagData {
    pub x:           f32,  // raw X (LSB, sensor frame)
    pub y:           f32,  // raw Y (LSB, sensor frame)
    pub z:           f32,  // raw Z (LSB, sensor frame)
    pub heading_rad: f32,  // tilt-compensated magnetic heading (0=N, clockwise +)
    pub valid:       bool,
}

/// Normalised servo positions [0.0, 1.0] written by navigation/ground station.
/// 0.0 = 1000 µs (min), 1.0 = 2000 µs (max). Defaults to 0.0 (retracted).
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct ServoOutputs {
    pub s1: f32,
    pub s2: f32,
    pub s3: f32,
    pub s4: f32,
}

/// Active weed pull target set by telemetry_task when a SET_POSITION_TARGET_LOCAL_NED
/// message arrives from the Pi. Cleared by navigation_task after servo actuation.
#[derive(Clone, Copy, defmt::Format)]
pub struct WeedTarget {
    pub position:     LatLonAlt,
    /// Barometer AGL altitude (metres) to descend to for extraction.
    /// Computed in telemetry_task as baro.altitude_m − ned_d at receive time.
    pub extract_alt_m: f32,
    pub valid:        bool,
}

impl Default for WeedTarget {
    fn default() -> Self {
        Self { position: LatLonAlt::default(), extract_alt_m: 0.4, valid: false }
    }
}

/// Payload presence flags — set by each payload task on successful hardware init.
/// Packed into HEARTBEAT custom_mode bits [31:16] and forwarded to the GCS.
pub mod payload_flags {
    pub const SERVO_OUTPUTS: u32 = 1 << 0;  // TIM4 servo bus (4 channels, PD12–PD15)
}

pub struct SharedState {
    pub attitude:      Mutex<CriticalSectionRawMutex, Quaternion>,
    pub imu_data:      Mutex<CriticalSectionRawMutex, ImuData>,
    pub baro_data:     Mutex<CriticalSectionRawMutex, BaroData>,
    pub rc_input:      Mutex<CriticalSectionRawMutex, RcInput>,
    pub nav_command:   Mutex<CriticalSectionRawMutex, NavCommand>,
    pub motor_outputs: Mutex<CriticalSectionRawMutex, MotorOutputs>,
    pub gps_fix:       Mutex<CriticalSectionRawMutex, GpsFix>,
    pub battery:       Mutex<CriticalSectionRawMutex, BatteryData>,
    pub armed:         Mutex<CriticalSectionRawMutex, bool>,
    pub flow:          Mutex<CriticalSectionRawMutex, FlowData>,
    pub servo_outputs: Mutex<CriticalSectionRawMutex, ServoOutputs>,
    pub mag_data:      Mutex<CriticalSectionRawMutex, MagData>,
    pub pos_estimate:  Mutex<CriticalSectionRawMutex, PosEstimate>,
    pub sensor_health: Mutex<CriticalSectionRawMutex, SensorHealth>,
    pub payload_flags:  Mutex<CriticalSectionRawMutex, u32>,
    pub weed_target:    Mutex<CriticalSectionRawMutex, WeedTarget>,
    /// Written by telemetry_task when MAV_CMD_DO_SET_HOME (179) arrives.
    /// Consumed (take()) by navigation_task to update mission.home.
    pub home_override:  Mutex<CriticalSectionRawMutex, Option<LatLonAlt>>,
    /// Bench motor-test override (MAV_CMD_DO_MOTOR_TEST). None = no test active.
    pub motor_test:     Mutex<CriticalSectionRawMutex, Option<MotorTest>>,
}

impl SharedState {
    pub const fn new() -> Self {
        Self {
            attitude:      Mutex::new(Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }),
            imu_data:      Mutex::new(ImuData    { accel: Vec3 { x:0.0,y:0.0,z:0.0 }, gyro: Vec3 { x:0.0,y:0.0,z:0.0 }, temp_c: 0.0 }),
            baro_data:     Mutex::new(BaroData   { pressure_pa: 101325.0, temp_c: 25.0, altitude_m: 0.0 }),
            rc_input:      Mutex::new(RcInput    { throttle:0.0, roll:0.0, pitch:0.0, yaw:0.0, arm:false, mode: FlightMode::Stabilise, failsafe: false }),
            nav_command:   Mutex::new(NavCommand { autonomous: false, attitude_setpoint: AttitudeSetpoint { roll:0.0, pitch:0.0, yaw_rate:0.0, throttle:0.0 }, target: LatLonAlt { lat_deg:0.0, lon_deg:0.0, alt_m:0.0 } }),
            motor_outputs: Mutex::new(MotorOutputs { m1:0.0, m2:0.0, m3:0.0, m4:0.0 }),
            gps_fix:       Mutex::new(GpsFix { lat_deg:0.0, lon_deg:0.0, alt_m:0.0, vel_n_ms:0.0, vel_e_ms:0.0, vel_d_ms:0.0, hacc_m:9999.0, fix_ok:false, fix_type:0 }),
            battery:       Mutex::new(BatteryData { voltage_v:0.0, pct:0, critical:true, current_a:-1.0 }),
            armed:         Mutex::new(false),
            flow:          Mutex::new(FlowData { quality:0, vel_x_mrad_s:0, vel_y_mrad_s:0, height_mm:-1, valid:false }),
            servo_outputs: Mutex::new(ServoOutputs { s1:0.0, s2:0.0, s3:0.0, s4:0.0 }),
            mag_data:      Mutex::new(MagData { x:0.0, y:0.0, z:0.0, heading_rad:0.0, valid:false }),
            pos_estimate:  Mutex::new(PosEstimate { pos_n:0.0, pos_e:0.0, pos_d:0.0, vel_n:0.0, vel_e:0.0, vel_d:0.0, valid:false }),
            sensor_health: Mutex::new(SensorHealth { imu_ok:false, baro_ok:false, mag_ok:false, gps_ok:false }),
            payload_flags:  Mutex::new(0u32),
            weed_target:    Mutex::new(WeedTarget { position: LatLonAlt { lat_deg:0.0, lon_deg:0.0, alt_m:0.0 }, extract_alt_m: 0.4, valid: false }),
            home_override:  Mutex::new(None),
            motor_test:     Mutex::new(None),
        }
    }
}
