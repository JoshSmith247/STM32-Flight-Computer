//! Shared data types and inter-task state.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

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
}

impl Default for BatteryData {
    fn default() -> Self {
        // Pessimistic defaults: GCS shows 0 V / critical until battery_task
        // confirms healthy voltage (~2.5 s after boot). Prevents the operator
        // from receiving a false "battery OK" before the first ADC reading.
        Self { voltage_v: 0.0, pct: 0, critical: true }
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
    Auto,           // Follow waypoint mission
    ReturnToHome,
    Land,
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

/// Motor mixer output for a quad in X configuration.
/// Motor order: front-right (CW), back-left (CW), front-left (CCW), back-right (CCW).
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct MotorOutputs {
    pub m1: f32,
    pub m2: f32,
    pub m3: f32,
    pub m4: f32,
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
    pub position: LatLonAlt,
    pub valid:    bool,
}

impl Default for WeedTarget {
    fn default() -> Self {
        Self { position: LatLonAlt::default(), valid: false }
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
    pub payload_flags: Mutex<CriticalSectionRawMutex, u32>,
    pub weed_target:   Mutex<CriticalSectionRawMutex, WeedTarget>,
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
            battery:       Mutex::new(BatteryData { voltage_v:0.0, pct:0, critical:true }),
            armed:         Mutex::new(false),
            flow:          Mutex::new(FlowData { quality:0, vel_x_mrad_s:0, vel_y_mrad_s:0, height_mm:-1, valid:false }),
            servo_outputs: Mutex::new(ServoOutputs { s1:0.0, s2:0.0, s3:0.0, s4:0.0 }),
            mag_data:      Mutex::new(MagData { x:0.0, y:0.0, z:0.0, heading_rad:0.0, valid:false }),
            payload_flags: Mutex::new(0u32),
            weed_target:   Mutex::new(WeedTarget { position: LatLonAlt { lat_deg:0.0, lon_deg:0.0, alt_m:0.0 }, valid: false }),
        }
    }
}
