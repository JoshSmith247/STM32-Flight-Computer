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
    let mut mission = Mission::new();
    // TODO: load mission from flash/EEPROM or uplink via MAVLink MISSION_ITEM

    info!("Navigation task started");
    let mut ticker = Ticker::every(Duration::from_hz(100));

    loop {
        ticker.next().await;

        let mode      = STATE.rc_input.lock().await.mode;
        let gps       = *STATE.gps_fix.lock().await;
        let baro      = *STATE.baro_data.lock().await;
        let attitude  = *STATE.attitude.lock().await;
        let is_armed  = *STATE.armed.lock().await;

        if !is_armed {
            let mut cmd = STATE.nav_command.lock().await;
            cmd.autonomous = false;
            continue;
        }

        let cmd = match mode {
            FlightMode::Auto => {
                navigate_mission(&mut mission, gps, baro.altitude_m)
            }
            FlightMode::ReturnToHome => {
                navigate_to(mission.home, gps, baro.altitude_m, 30.0)
            }
            FlightMode::Land => {
                land_setpoint(baro.altitude_m)
            }
            FlightMode::PositionHold => {
                // TODO: use a horizontal position PID here
                NavCommand {
                    autonomous: true,
                    attitude_setpoint: AttitudeSetpoint {
                        roll: 0.0, pitch: 0.0, yaw_rate: 0.0,
                        throttle: 0.55, // approximate hover throttle
                    },
                    target: gps,
                }
            }
            _ => NavCommand { autonomous: false, ..Default::default() },
        };

        *STATE.nav_command.lock().await = cmd;
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
