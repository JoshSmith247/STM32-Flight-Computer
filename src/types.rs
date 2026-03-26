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

pub struct SharedState {
    pub attitude:     Mutex<CriticalSectionRawMutex, Quaternion>,
    pub imu_data:     Mutex<CriticalSectionRawMutex, ImuData>,
    pub baro_data:    Mutex<CriticalSectionRawMutex, BaroData>,
    pub rc_input:     Mutex<CriticalSectionRawMutex, RcInput>,
    pub nav_command:  Mutex<CriticalSectionRawMutex, NavCommand>,
    pub motor_outputs:Mutex<CriticalSectionRawMutex, MotorOutputs>,
    pub gps_fix:      Mutex<CriticalSectionRawMutex, LatLonAlt>,
    pub armed:        Mutex<CriticalSectionRawMutex, bool>,
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
            gps_fix:       Mutex::new(LatLonAlt  { lat_deg:0.0, lon_deg:0.0, alt_m:0.0 }),
            armed:         Mutex::new(false),
        }
    }
}
