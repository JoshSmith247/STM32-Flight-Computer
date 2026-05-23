//! Autonomous navigation — waypoint sequencer and flight mode logic.
//!
//! Runs at 100 Hz. Reads GPS position + barometer altitude, sequences through
//! a pre-loaded mission, and writes attitude setpoints to STATE.nav_command.
//!
//! Supported autonomous modes:
//!   Auto      — fly a waypoint mission
//!   ReturnToHome (RTH) — fly back to home position and land
//!   Land      — descend at fixed rate until touchdown
//!   PositionHold — hold current GPS position

use defmt::{info, warn};
use embassy_time::{Duration, Ticker};
use libm::{atan2f, cosf, sinf, sqrtf};

use crate::{
    pid::{AltPid, PosPid},
    state,
    types::{AttitudeSetpoint, FlightMode, LatLonAlt, NavCommand},
    STATE,
};

const MAX_WAYPOINTS: usize = 32;
const WAYPOINT_RADIUS_M: f32 = 2.0;   // accept radius around waypoint
const CRUISE_SPEED_MPS:  f32 = 5.0;   // m/s horizontal
const LAND_DESCENT_MPS:  f32 = 0.5;   // m/s descent rate
const MAX_TILT_RAD:      f32 = 0.436; // 25 deg max lean

#[derive(Clone, Copy, Default)]
pub struct Waypoint {
    pub position: LatLonAlt,
    pub hold_time_s: f32,    // hover time before advancing (0 = fly-through)
}

pub struct Mission {
    waypoints:  [Waypoint; MAX_WAYPOINTS],
    count:      usize,
    current:    usize,
    home:       LatLonAlt,
}

impl Mission {
    pub const fn new() -> Self {
        Self {
            waypoints: [Waypoint { position: LatLonAlt { lat_deg: 0.0, lon_deg: 0.0, alt_m: 0.0 }, hold_time_s: 0.0 }; MAX_WAYPOINTS],
            count: 0,
            current: 0,
            home: LatLonAlt { lat_deg: 0.0, lon_deg: 0.0, alt_m: 0.0 },
        }
    }

    pub fn add_waypoint(&mut self, wp: Waypoint) {
        if self.count < MAX_WAYPOINTS {
            self.waypoints[self.count] = wp;
            self.count += 1;
        }
    }

    pub fn set_home(&mut self, pos: LatLonAlt) {
        self.home = pos;
    }

    pub fn current_target(&self) -> Option<&Waypoint> {
        if self.current < self.count { Some(&self.waypoints[self.current]) } else { None }
    }

    pub fn advance(&mut self) {
        self.current = (self.current + 1).min(self.count);
    }

    pub fn is_complete(&self) -> bool {
        self.current >= self.count
    }
}

/// Haversine distance between two GPS coordinates (metres).
pub fn haversine_m(a: LatLonAlt, b: LatLonAlt) -> f32 {
    const R: f32 = 6_371_000.0; // Earth radius in metres
    let lat1 = (a.lat_deg as f32).to_radians();
    let lat2 = (b.lat_deg as f32).to_radians();
    let dlat = ((b.lat_deg - a.lat_deg) as f32).to_radians();
    let dlon = ((b.lon_deg - a.lon_deg) as f32).to_radians();

    let sin_dlat = sinf(dlat / 2.0);
    let sin_dlon = sinf(dlon / 2.0);
    let a_val = sin_dlat*sin_dlat + cosf(lat1)*cosf(lat2)*sin_dlon*sin_dlon;
    2.0 * R * libm::asinf(sqrtf(a_val.min(1.0)))
}

/// Bearing from `from` to `to` (radians, 0 = North, clockwise positive).
pub fn bearing_rad(from: LatLonAlt, to: LatLonAlt) -> f32 {
    let lat1 = (from.lat_deg as f32).to_radians();
    let lat2 = (to.lat_deg   as f32).to_radians();
    let dlon = ((to.lon_deg - from.lon_deg) as f32).to_radians();
    atan2f(sinf(dlon)*cosf(lat2), cosf(lat1)*sinf(lat2) - sinf(lat1)*cosf(lat2)*cosf(dlon))
}

// ---------------------------------------------------------------------------
// Embassy task
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn navigation_task() {
    let mut mission   = Mission::new();
    let mut alt_pid   = AltPid::new();
    let mut pos_pid_n = PosPid::new();     // NED North axis
    let mut pos_pid_e = PosPid::new();     // NED East axis
    let mut hold_alt: f32     = 0.0;
    let mut hold_pos          = LatLonAlt::default();
    let mut prev_mode         = FlightMode::Stabilise;
    let mut home_set          = false;

    info!("Navigation task started");
    let mut ticker = Ticker::every(Duration::from_hz(100));

    loop {
        ticker.next().await;

        let mode     = STATE.rc_input.lock().await.mode;
        let gps      = *STATE.gps_fix.lock().await;
        let baro     = *STATE.baro_data.lock().await;
        let attitude = *STATE.attitude.lock().await;
        let battery  = *STATE.battery.lock().await;
        let is_armed = *STATE.armed.lock().await;

        if !is_armed {
            STATE.nav_command.lock().await.autonomous = false;
            continue;
        }

        // Capture home position on first confirmed 3-D fix
        if !home_set && gps.fix_ok {
            mission.set_home(gps.pos());
            home_set = true;
            info!("Home set: lat={=f64} lon={=f64}", gps.lat_deg, gps.lon_deg);
        }

        // Fault state (Pi lost, or battery task set it) → force Land immediately.
        // Auto-disarm once we touch down so the drone can't re-arm from Fault.
        if state::get() == state::FlightState::Fault {
            if baro.altitude_m < 0.3 && is_armed {
                *STATE.armed.lock().await = false;
                info!("Fault landing complete — disarmed");
            }
        }

        // Battery critical abort — override RC mode with RTH or Land.
        // Keeps the current mode if we are already landing.
        let effective_mode = if state::get() == state::FlightState::Fault {
            FlightMode::Land
        } else if battery.critical && !matches!(mode, FlightMode::Land) {
            if gps.fix_ok && home_set {
                warn!("Critical battery — forcing RTH");
                state::set(state::FlightState::Landing);
                FlightMode::ReturnToHome
            } else {
                warn!("Critical battery — forcing Land");
                state::set(state::FlightState::Landing);
                FlightMode::Land
            }
        } else {
            mode
        };

        let cmd = match effective_mode {
            FlightMode::Auto => {
                if !gps.fix_ok {
                    warn!("Auto: no GPS fix — loitering in place");
                    NavCommand { autonomous: false, ..Default::default() }
                } else {
                    navigate_mission(&mut mission, gps.pos(), baro.altitude_m)
                }
            }
            FlightMode::ReturnToHome => {
                if !gps.fix_ok || !home_set {
                    warn!("RTH: no GPS fix or home — forcing Land");
                    land_setpoint(baro.altitude_m)
                } else {
                    navigate_to(mission.home, gps.pos(), baro.altitude_m, 30.0)
                }
            }
            FlightMode::Land => {
                land_setpoint(baro.altitude_m)
            }
            FlightMode::PositionHold => {
                if prev_mode != FlightMode::PositionHold {
                    hold_alt  = baro.altitude_m;
                    hold_pos  = gps.pos();
                    pos_pid_n.reset();
                    pos_pid_e.reset();
                    alt_pid.reset();
                    info!("PositionHold: alt={=f32}m  lat={=f64}  lon={=f64}",
                          hold_alt, hold_pos.lat_deg, hold_pos.lon_deg);
                }

                let (roll_sp, pitch_sp) = if gps.fix_ok {
                    // Position error in NED metres (f64 for precision, cast before PIDs)
                    let lat_rad = (gps.lat_deg as f32).to_radians();
                    let err_n   = ((hold_pos.lat_deg - gps.lat_deg) * 111_320.0) as f32;
                    let err_e   = ((hold_pos.lon_deg - gps.lon_deg) * 111_320.0
                                   * cosf(lat_rad) as f64) as f32;

                    // Rotate NED error into body frame using drone heading
                    let yaw = atan2f(
                        2.0 * (attitude.w * attitude.z + attitude.x * attitude.y),
                        1.0 - 2.0 * (attitude.y * attitude.y + attitude.z * attitude.z),
                    );
                    let fwd_m   =  err_n * cosf(yaw) + err_e * sinf(yaw);
                    let right_m = -err_n * sinf(yaw) + err_e * cosf(yaw);

                    // Pitch nose-down to go forward (negative pitch = forward).
                    // Roll right to go east (positive roll = right).
                    let pitch = -pos_pid_n.update(fwd_m);
                    let roll  =  pos_pid_e.update(right_m);
                    (roll, pitch)
                } else {
                    // No GPS fix — altitude hold only, zero horizontal lean
                    (0.0, 0.0)
                };

                NavCommand {
                    autonomous: true,
                    attitude_setpoint: AttitudeSetpoint {
                        roll:     roll_sp,
                        pitch:    pitch_sp,
                        yaw_rate: 0.0,
                        throttle: alt_pid.update(hold_alt, baro.altitude_m),
                    },
                    target: hold_pos,
                }
            }
            _ => NavCommand { autonomous: false, ..Default::default() },
        };

        *STATE.nav_command.lock().await = cmd;
        prev_mode = effective_mode;
    }
}

fn navigate_mission(mission: &mut Mission, pos: LatLonAlt, alt_m: f32) -> NavCommand {
    if mission.is_complete() {
        info!("Mission complete — loitering");
        return NavCommand {
            autonomous: true,
            attitude_setpoint: AttitudeSetpoint { roll:0.0, pitch:0.0, yaw_rate:0.0, throttle:0.55 },
            target: pos,
        };
    }

    let target = *mission.current_target().unwrap();
    let dist = haversine_m(pos, target.position);

    if dist < WAYPOINT_RADIUS_M {
        info!("Waypoint {} reached", mission.current);
        mission.advance();
    }

    navigate_to(target.position, pos, alt_m, target.position.alt_m)
}

fn navigate_to(target: LatLonAlt, pos: LatLonAlt, current_alt: f32, target_alt: f32) -> NavCommand {
    let dist    = haversine_m(pos, target);
    let bearing = bearing_rad(pos, target);

    // Simple proportional controller: lean towards target
    let lean = (dist / 20.0).min(1.0) * MAX_TILT_RAD; // full lean at ≥20 m away
    let roll_sp  =  sinf(bearing) * lean;
    let pitch_sp = -cosf(bearing) * lean; // nose-down = forward

    // Altitude hold: proportional throttle correction
    let alt_error   = target_alt - current_alt;
    let throttle_sp = (0.55 + alt_error * 0.02).clamp(0.1, 0.9);

    NavCommand {
        autonomous: true,
        attitude_setpoint: AttitudeSetpoint {
            roll:     roll_sp,
            pitch:    pitch_sp,
            yaw_rate: 0.0,
            throttle: throttle_sp,
        },
        target,
    }
}

fn land_setpoint(current_alt: f32) -> NavCommand {
    // Descend at LAND_DESCENT_MPS; disarm near ground
    let throttle = if current_alt < 0.3 { 0.0 } else { 0.52 };
    NavCommand {
        autonomous: true,
        attitude_setpoint: AttitudeSetpoint { roll:0.0, pitch:0.0, yaw_rate:0.0, throttle },
        target: Default::default(),
    }
}
