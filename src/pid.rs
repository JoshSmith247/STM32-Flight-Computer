//! PID controller cascade: rate → attitude → [altitude/position].
//!
//! The drone uses a two-stage angular cascade:
//!   Outer loop (attitude):  setpoint angle → desired body rate
//!   Inner loop (rate):      desired body rate → motor torque demand
//!
//! All loops run at 500 Hz from `control_task` in main.rs.

use crate::types::{AttitudeSetpoint, Euler, MotorOutputs, Vec3};

// ---------------------------------------------------------------------------
// Single-axis PID state
// ---------------------------------------------------------------------------

pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    integral:  f32,
    prev_error: f32,
    i_limit:   f32,    // anti-windup clamp
    output_limit: f32,
}

impl Pid {
    pub const fn new(kp: f32, ki: f32, kd: f32, i_limit: f32, output_limit: f32) -> Self {
        Self { kp, ki, kd, integral: 0.0, prev_error: 0.0, i_limit, output_limit }
    }

    /// Run one PID step. `dt` in seconds.
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;

        self.integral = (self.integral + error * dt).clamp(-self.i_limit, self.i_limit);

        let derivative = if dt > 0.0 { (error - self.prev_error) / dt } else { 0.0 };
        self.prev_error = error;

        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        output.clamp(-self.output_limit, self.output_limit)
    }

    pub fn reset(&mut self) {
        self.integral   = 0.0;
        self.prev_error = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Full three-axis PID cascade
// ---------------------------------------------------------------------------

/// Default tuning — PLACEHOLDER values, must be tuned on actual hardware.
const DT: f32 = 1.0 / 500.0;

pub struct FlightPids {
    // Outer attitude loop (angle → rate setpoint)
    pub att_roll:  Pid,
    pub att_pitch: Pid,
    pub att_yaw:   Pid,

    // Inner rate loop (rate → torque)
    pub rate_roll:  Pid,
    pub rate_pitch: Pid,
    pub rate_yaw:   Pid,
}

impl Default for FlightPids {
    fn default() -> Self {
        Self {
            att_roll:   Pid::new(4.5, 0.0, 0.0,  100.0, 400.0),
            att_pitch:  Pid::new(4.5, 0.0, 0.0,  100.0, 400.0),
            att_yaw:    Pid::new(4.0, 0.0, 0.0,  100.0, 400.0),

            rate_roll:  Pid::new(60.0, 40.0, 2.5, 200.0, 500.0),
            rate_pitch: Pid::new(60.0, 40.0, 2.5, 200.0, 500.0),
            rate_yaw:   Pid::new(80.0, 30.0, 0.0, 200.0, 500.0),
        }
    }
}

impl FlightPids {
    /// Full cascade: attitude setpoint + current attitude + body rates → motor mix inputs.
    ///
    /// Returns (roll_torque, pitch_torque, yaw_torque) normalised ±1.0.
    pub fn update(
        &mut self,
        setpoint: &AttitudeSetpoint,
        attitude: &Euler,
        body_rate: Vec3,
    ) -> (f32, f32, f32) {
        // --- Outer loop: angle error → rate setpoint ---
        let roll_rate_sp  = self.att_roll.update(setpoint.roll,  attitude.roll,  DT);
        let pitch_rate_sp = self.att_pitch.update(setpoint.pitch, attitude.pitch, DT);
        let yaw_rate_sp   = setpoint.yaw_rate; // direct rate control on yaw

        // --- Inner loop: rate error → torque demand ---
        let roll_out  = self.rate_roll.update( roll_rate_sp,  body_rate.x, DT);
        let pitch_out = self.rate_pitch.update(pitch_rate_sp, body_rate.y, DT);
        let yaw_out   = self.rate_yaw.update(  yaw_rate_sp,   body_rate.z, DT);

        (roll_out / 500.0, pitch_out / 500.0, yaw_out / 500.0)
    }

    pub fn reset_all(&mut self) {
        self.att_roll.reset();  self.att_pitch.reset();  self.att_yaw.reset();
        self.rate_roll.reset(); self.rate_pitch.reset(); self.rate_yaw.reset();
    }
}

// ---------------------------------------------------------------------------
// Motor mixer — quad X configuration
// ---------------------------------------------------------------------------
//
//   Front
//  2(CCW) 1(CW)
//  3(CW)  4(CCW)
//   Back
//
//   Motor  Roll  Pitch  Yaw
//     1    -1    +1     -1   (front-right, CW)
//     2    +1    +1     +1   (front-left,  CCW)
//     3    -1    -1     +1   (back-right,  CCW)
//     4    +1    -1     -1   (back-left,   CW)

pub fn mix_quad_x(throttle: f32, roll: f32, pitch: f32, yaw: f32) -> MotorOutputs {
    let m1 = (throttle - roll + pitch - yaw).clamp(0.0, 1.0);
    let m2 = (throttle + roll + pitch + yaw).clamp(0.0, 1.0);
    let m3 = (throttle - roll - pitch + yaw).clamp(0.0, 1.0);
    let m4 = (throttle + roll - pitch - yaw).clamp(0.0, 1.0);
    MotorOutputs { m1, m2, m3, m4 }
}
