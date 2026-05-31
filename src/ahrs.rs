//! Madgwick AHRS filter.
//!
//! Fuses gyroscope and accelerometer data into a unit quaternion attitude
//! estimate. At 2000 Hz with beta ≈ 0.1 the filter converges in ~2 s.
//!
//! Reference: Madgwick, S. (2010). "An efficient orientation filter for
//! inertial and inertial/magnetic sensor arrays."

use libm::{fabsf, sqrtf};

use crate::types::{Euler, Quaternion, Vec3};

pub struct MadgwickFilter {
    beta: f32,          // Filter gain (higher = faster convergence, noisier)
    dt:   f32,          // Sample period in seconds
    q:    Quaternion,   // Current attitude estimate
}

impl MadgwickFilter {
    /// `beta` ≈ 0.1 for IMU-only; `dt` = 1.0 / sample_rate_hz.
    pub fn new(beta: f32, dt: f32) -> Self {
        Self { beta, dt, q: Quaternion::default() }
    }

    /// Update filter with gyro (rad/s) and accel (m/s² or any unit — normalised internally).
    /// Returns updated attitude quaternion.
    pub fn update(&mut self, gyro: Vec3, accel: Vec3) -> Quaternion {
        let (mut q0, mut q1, mut q2, mut q3) = (self.q.w, self.q.x, self.q.y, self.q.z);

        // Normalise accelerometer
        let a_norm = 1.0 / sqrtf(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
        if !a_norm.is_finite() || fabsf(a_norm) < 1e-10 { return self.q; }
        let (ax, ay, az) = (accel.x * a_norm, accel.y * a_norm, accel.z * a_norm);

        // Gradient descent correction (objective function for gravity)
        let f1 = 2.0*(q1*q3 - q0*q2) - ax;
        let f2 = 2.0*(q0*q1 + q2*q3) - ay;
        let f3 = 2.0*(0.5 - q1*q1 - q2*q2) - az;

        // Gradient = J_g^T * f_g (see Madgwick 2010, eq. 35)
        let s0 = -2.0*q2*f1 + 2.0*q1*f2;
        let s1 =  2.0*q3*f1 + 2.0*q0*f2 - 4.0*q1*f3;
        let s2 = -2.0*q0*f1 + 2.0*q3*f2 - 4.0*q2*f3;
        let s3 =  2.0*q1*f1 + 2.0*q2*f2;

        // Normalise gradient
        let s_norm = 1.0 / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        let (s0, s1, s2, s3) = (s0*s_norm, s1*s_norm, s2*s_norm, s3*s_norm);

        // Rate of change of quaternion from gyro
        let qd0 = 0.5*(-q1*gyro.x - q2*gyro.y - q3*gyro.z);
        let qd1 = 0.5*( q0*gyro.x + q2*gyro.z - q3*gyro.y);
        let qd2 = 0.5*( q0*gyro.y - q1*gyro.z + q3*gyro.x);
        let qd3 = 0.5*( q0*gyro.z + q1*gyro.y - q2*gyro.x);

        // Integrate
        q0 += (qd0 - self.beta*s0) * self.dt;
        q1 += (qd1 - self.beta*s1) * self.dt;
        q2 += (qd2 - self.beta*s2) * self.dt;
        q3 += (qd3 - self.beta*s3) * self.dt;

        // Normalise quaternion
        let norm = 1.0 / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        self.q = Quaternion { w: q0*norm, x: q1*norm, y: q2*norm, z: q3*norm };
        self.q
    }

    /// Update filter with gyro (rad/s), accel, and magnetometer readings (9-DOF).
    /// Adds a magnetic field gradient correction to the gravity correction so
    /// heading converges to compass North rather than drifting with gyro bias.
    /// Falls back to 6-DOF if the magnetometer vector is zero or denormal.
    ///
    /// Reference: Madgwick 2010, Section 7.1 — gradient for combined gravity+mag objective.
    pub fn update_with_mag(&mut self, gyro: Vec3, accel: Vec3, mag: Vec3) -> Quaternion {
        let (mut q0, mut q1, mut q2, mut q3) = (self.q.w, self.q.x, self.q.y, self.q.z);

        // Normalise accelerometer
        let a_norm = 1.0 / sqrtf(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
        if !a_norm.is_finite() || fabsf(a_norm) < 1e-10 { return self.q; }
        let (ax, ay, az) = (accel.x * a_norm, accel.y * a_norm, accel.z * a_norm);

        // Normalise magnetometer — fall back to 6-DOF if invalid
        let m_norm = 1.0 / sqrtf(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);
        if !m_norm.is_finite() || fabsf(m_norm) < 1e-10 { return self.update(gyro, accel); }
        let (mx, my, mz) = (mag.x * m_norm, mag.y * m_norm, mag.z * m_norm);

        // Earth frame magnetic field h = q ⊗ m_body ⊗ q*
        let hx = 2.0*mx*(0.5 - q2*q2 - q3*q3) + 2.0*my*(q1*q2 - q0*q3) + 2.0*mz*(q1*q3 + q0*q2);
        let hy = 2.0*mx*(q0*q3 + q1*q2) + 2.0*my*(0.5 - q1*q1 - q3*q3) + 2.0*mz*(q2*q3 - q0*q1);
        let hz = 2.0*mx*(q1*q3 - q0*q2) + 2.0*my*(q0*q1 + q2*q3) + 2.0*mz*(0.5 - q1*q1 - q2*q2);

        // Reference earth field: project h onto XZ plane (eliminate Y component).
        // bx = horizontal flux magnitude, bz = vertical flux (inclination).
        let bx = sqrtf(hx*hx + hy*hy);
        let bz = hz;

        // Gravity objective function  fg = f(q, g_ref) - a_measured
        let fg0 = 2.0*(q1*q3 - q0*q2) - ax;
        let fg1 = 2.0*(q0*q1 + q2*q3) - ay;
        let fg2 = 2.0*(0.5 - q1*q1 - q2*q2) - az;

        // Magnetic objective function  fb = f(q, b_ref) - m_measured
        let fb0 = 2.0*bx*(0.5 - q2*q2 - q3*q3) + 2.0*bz*(q1*q3 - q0*q2) - mx;
        let fb1 = 2.0*bx*(q1*q2 - q0*q3)        + 2.0*bz*(q0*q1 + q2*q3) - my;
        let fb2 = 2.0*bx*(q0*q2 + q1*q3)        + 2.0*bz*(0.5 - q1*q1 - q2*q2) - mz;

        // Combined gradient  s = J_g^T * fg + J_b^T * fb
        let s0 = (-2.0*q2*fg0 + 2.0*q1*fg1)
               + (-2.0*bz*q2*fb0 + (-2.0*bx*q3 + 2.0*bz*q1)*fb1 + 2.0*bx*q2*fb2);
        let s1 = (2.0*q3*fg0 + 2.0*q0*fg1 - 4.0*q1*fg2)
               + (2.0*bz*q3*fb0 + (2.0*bx*q2 + 2.0*bz*q0)*fb1 + (2.0*bx*q3 - 4.0*bz*q1)*fb2);
        let s2 = (-2.0*q0*fg0 + 2.0*q3*fg1 - 4.0*q2*fg2)
               + ((-4.0*bx*q2 - 2.0*bz*q0)*fb0 + (2.0*bx*q1 + 2.0*bz*q3)*fb1 + (2.0*bx*q0 - 4.0*bz*q2)*fb2);
        let s3 = (2.0*q1*fg0 + 2.0*q2*fg1)
               + ((-4.0*bx*q3 + 2.0*bz*q1)*fb0 + (-2.0*bx*q0 + 2.0*bz*q2)*fb1 + 2.0*bx*q1*fb2);

        // Normalise gradient
        let s_norm = 1.0 / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        let (s0, s1, s2, s3) = (s0*s_norm, s1*s_norm, s2*s_norm, s3*s_norm);

        // Rate of change from gyro
        let qd0 = 0.5*(-q1*gyro.x - q2*gyro.y - q3*gyro.z);
        let qd1 = 0.5*( q0*gyro.x + q2*gyro.z - q3*gyro.y);
        let qd2 = 0.5*( q0*gyro.y - q1*gyro.z + q3*gyro.x);
        let qd3 = 0.5*( q0*gyro.z + q1*gyro.y - q2*gyro.x);

        // Integrate
        q0 += (qd0 - self.beta*s0) * self.dt;
        q1 += (qd1 - self.beta*s1) * self.dt;
        q2 += (qd2 - self.beta*s2) * self.dt;
        q3 += (qd3 - self.beta*s3) * self.dt;

        // Normalise quaternion
        let norm = 1.0 / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        self.q = Quaternion { w: q0*norm, x: q1*norm, y: q2*norm, z: q3*norm };
        self.q
    }

    /// Convert current quaternion to Euler angles (roll, pitch, yaw) in radians.
    pub fn euler(&self) -> Euler {
        let q = &self.q;
        Euler {
            roll:  libm::atan2f(2.0*(q.w*q.x + q.y*q.z), 1.0 - 2.0*(q.x*q.x + q.y*q.y)),
            pitch: libm::asinf((2.0*(q.w*q.y - q.z*q.x)).clamp(-1.0, 1.0)),
            yaw:   libm::atan2f(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z)),
        }
    }
}
