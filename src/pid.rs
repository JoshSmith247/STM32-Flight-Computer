//! PID controller cascade: outer attitude loop (angle to desired body rate) feeding
//! an inner rate loop (rate to torque demand), 500 Hz from `control_task`.

use crate::types::{AttitudeSetpoint, Euler, MotorOutputs, Vec3};

// Gyro low-pass filter (PT1)

/// First-order (PT1) low-pass applied to the gyro before the rate PID to attenuate
/// motor noise on the D-term. cutoff_hz trades noise rejection against phase lag.
#[derive(Clone, Copy)]
pub struct LowPass3 {
    alpha: f32,
    y:     Vec3,
}

impl LowPass3 {
    pub fn new(cutoff_hz: f32, dt: f32) -> Self {
        let rc = 1.0 / (2.0 * core::f32::consts::PI * cutoff_hz);
        Self { alpha: dt / (dt + rc), y: Vec3 { x: 0.0, y: 0.0, z: 0.0 } }
    }

    /// Update with a new sample and return the filtered value.
    pub fn apply(&mut self, x: Vec3) -> Vec3 {
        self.y.x += self.alpha * (x.x - self.y.x);
        self.y.y += self.alpha * (x.y - self.y.y);
        self.y.z += self.alpha * (x.z - self.y.z);
        self.y
    }
}

// Single-axis PID state

pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    integral:         f32,
    prev_measurement: f32,
    prev_derivative:  f32,    // held D-term while the measurement is unchanged
    stale_ticks:      u32,    // updates since the measurement last changed
    primed:           bool,   // false until the first sample after reset
    i_limit:          f32,    // anti-windup clamp
    output_limit:     f32,
}

impl Pid {
    pub const fn new(kp: f32, ki: f32, kd: f32, i_limit: f32, output_limit: f32) -> Self {
        Self { kp, ki, kd, integral: 0.0, prev_measurement: 0.0,
               prev_derivative: 0.0, stale_ticks: 0, primed: false,
               i_limit, output_limit }
    }

    /// Run one PID step. D acts on -d(measurement) (no setpoint kick), recomputed
    /// only when the measurement changes and held between - per-tick differencing
    /// would spike on slow sources (5 Hz GPS, 25 Hz baro).
    pub fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;

        self.integral = (self.integral + error * dt).clamp(-self.i_limit, self.i_limit);

        let derivative = if dt <= 0.0 {
            0.0
        } else if !self.primed {
            // First sample after reset: no history, no kick.
            self.prev_measurement = measurement;
            self.primed = true;
            self.stale_ticks = 0;
            self.prev_derivative = 0.0;
            0.0
        } else if measurement != self.prev_measurement {
            let span = dt * (self.stale_ticks + 1) as f32;
            let d = -(measurement - self.prev_measurement) / span;
            self.prev_measurement = measurement;
            self.stale_ticks = 0;
            self.prev_derivative = d;
            d
        } else {
            // Measurement unchanged - hold the last derivative.
            self.stale_ticks = self.stale_ticks.saturating_add(1);
            self.prev_derivative
        };

        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        output.clamp(-self.output_limit, self.output_limit)
    }

    pub fn reset(&mut self) {
        self.integral         = 0.0;
        self.prev_measurement = 0.0;
        self.prev_derivative  = 0.0;
        self.stale_ticks      = 0;
        self.primed           = false;
    }
}

// Full three-axis PID cascade

/// Conservative first-flight starting values for a mid-size quad. Tuning order:
/// rate_Kp until oscillation, back off 30%, then rate_Kd, rate_Ki, then attitude.
const DT: f32 = 1.0 / 500.0;
const RATE_OUTPUT_LIMIT: f32 = 500.0; // rate PID ceiling; also used to normalise to +/-1.0

pub struct FlightPids {
    // Outer attitude loop (angle -> rate setpoint)
    pub att_roll:  Pid,
    pub att_pitch: Pid,
    pub att_yaw:   Pid,

    // Inner rate loop (rate -> torque)
    pub rate_roll:  Pid,
    pub rate_pitch: Pid,
    pub rate_yaw:   Pid,
}

impl Default for FlightPids {
    fn default() -> Self {
        Self {
            // Outer loop: angle error (rad) -> rate setpoint (rad/s).
            // Ki/Kd left at zero - P-only is sufficient for the attitude shell.
            att_roll:   Pid::new(4.0, 0.0, 0.0, 100.0, 400.0),
            att_pitch:  Pid::new(4.0, 0.0, 0.0, 100.0, 400.0),
            att_yaw:    Pid::new(3.0, 0.0, 0.0, 100.0, 400.0),

            // Inner loop: rate error -> torque demand. Kd on yaw omitted (z-gyro noise);
            // i_limit = output_limit / Ki so the I-term alone can't saturate the output.
            rate_roll:  Pid::new(50.0, 30.0, 2.0,  16.0, RATE_OUTPUT_LIMIT),
            rate_pitch: Pid::new(50.0, 30.0, 2.0,  16.0, RATE_OUTPUT_LIMIT),
            rate_yaw:   Pid::new(70.0, 20.0, 0.0,  25.0, RATE_OUTPUT_LIMIT),
        }
    }
}

impl FlightPids {
    /// Full cascade: attitude setpoint + current attitude + body rates -> motor mix inputs.
    /// Returns (roll_torque, pitch_torque, yaw_torque) normalised +/-1.0.
    pub fn update(
        &mut self,
        setpoint: &AttitudeSetpoint,
        attitude: &Euler,
        body_rate: Vec3,
    ) -> (f32, f32, f32) {
        // --- Outer loop: angle error -> rate setpoint ---
        let roll_rate_sp  = self.att_roll.update(setpoint.roll,  attitude.roll,  DT);
        let pitch_rate_sp = self.att_pitch.update(setpoint.pitch, attitude.pitch, DT);
        let yaw_rate_sp   = setpoint.yaw_rate; // direct rate control on yaw

        // --- Inner loop: rate error -> torque demand ---
        let roll_out  = self.rate_roll.update( roll_rate_sp,  body_rate.x, DT);
        let pitch_out = self.rate_pitch.update(pitch_rate_sp, body_rate.y, DT);
        let yaw_out   = self.rate_yaw.update(  yaw_rate_sp,   body_rate.z, DT);

        (roll_out / RATE_OUTPUT_LIMIT, pitch_out / RATE_OUTPUT_LIMIT, yaw_out / RATE_OUTPUT_LIMIT)
    }

    pub fn reset_all(&mut self) {
        self.att_roll.reset();  self.att_pitch.reset();  self.att_yaw.reset();
        self.rate_roll.reset(); self.rate_pitch.reset(); self.rate_yaw.reset();
    }
}

// Altitude-hold PID - 100 Hz from navigation_task

pub struct AltPid(Pid);

impl AltPid {
    pub fn new() -> Self {
        // 1 m error -> 0.05 throttle nudge. i_limit clamps the INTEGRAL, not the
        // I output: Ki x 40 = 0.2 throttle of I authority for hover-throttle trim.
        Self(Pid::new(0.05, 0.005, 0.10, 40.0, 0.4))
    }

    /// Returns throttle [0.1, 0.9] to hold `target_m` above takeoff.
    pub fn update(&mut self, target_m: f32, current_m: f32) -> f32 {
        const DT: f32 = 1.0 / 100.0;
        const HOVER: f32 = 0.55;
        (HOVER + self.0.update(target_m, current_m, DT)).clamp(0.1, 0.9)
    }

    pub fn reset(&mut self) { self.0.reset(); }
}

// Position-hold PID - one per horizontal axis (N and E), 100 Hz

pub struct PosPid(Pid);

impl PosPid {
    pub fn new() -> Self {
        // 1 m error -> 0.05 rad lean, clamped to MAX_TILT_RAD. i_limit clamps the
        // INTEGRAL: Ki x 75 = 0.15 rad of I authority against steady wind.
        Self(Pid::new(0.05, 0.002, 0.08, 75.0, 0.436))
    }

    /// `err_m`: (target - current) in metres along one NED body-frame axis.
    /// Returns a lean angle in radians; sign convention: positive = lean positive.
    pub fn update(&mut self, err_m: f32) -> f32 {
        const DT: f32 = 1.0 / 100.0;
        // Pass error as -measurement so Pid's derivative-on-measurement gives
        // d(err)/dt, which damps oscillation. Setpoint=0 keeps P+I identical.
        self.0.update(0.0, -err_m, DT)
    }

    pub fn reset(&mut self) { self.0.reset(); }
}

// Motor mixer - quad X: M1 front-right CW, M2 front-left CCW,
// M3 back-right CCW, M4 back-left CW.

pub fn mix_quad_x(throttle: f32, roll: f32, pitch: f32, yaw: f32) -> MotorOutputs {
    let r1 = throttle - roll + pitch - yaw;
    let r2 = throttle + roll + pitch + yaw;
    let r3 = throttle - roll - pitch + yaw;
    let r4 = throttle + roll - pitch - yaw;

    // Shift all motors by the same offset before clamping so torque ratios are
    // preserved when any output would otherwise exceed [0, 1].
    let max_out = r1.max(r2).max(r3).max(r4);
    let min_out = r1.min(r2).min(r3).min(r4);
    let offset = if max_out > 1.0 { max_out - 1.0 } else if min_out < 0.0 { min_out } else { 0.0 };

    MotorOutputs {
        m1: (r1 - offset).clamp(0.0, 1.0),
        m2: (r2 - offset).clamp(0.0, 1.0),
        m3: (r3 - offset).clamp(0.0, 1.0),
        m4: (r4 - offset).clamp(0.0, 1.0),
    }
}
