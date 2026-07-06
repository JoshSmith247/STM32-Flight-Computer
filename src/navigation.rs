//! Autonomous navigation — flight mode logic and waypoint sequencer.
//!
//! Runs at 100 Hz. Writes attitude setpoints to STATE.nav_command.
//!
//! Modes:
//!   Stabilise    — RC direct control (nav_command.autonomous = false)
//!   AltitudeHold — baro altitude lock; RC roll/pitch/yaw pass-through
//!   PositionHold — GPS + baro lock; RC yaw pass-through
//!   Auto         — fly a MAVLink mission loaded via PENDING_MISSION
//!   ReturnToHome — fly to home at safe altitude, then descend
//!   Land         — rate-controlled descent; rangefinder-assisted touch-down

use defmt::{info, warn};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Instant, Ticker};
use libm::{atan2f, cosf, sinf, sqrtf};

use crate::{
    pid::{AltPid, PosPid},
    state,
    types::{AttitudeSetpoint, FlightMode, FlowData, LatLonAlt, NavCommand},
    STATE,
};

// ── Weed extraction phases ────────────────────────────────────────────────────

/// Seven-phase sequence executed each time a weed target is received in Auto mode.
#[derive(Clone, Copy, PartialEq)]
enum WeedPhase {
    /// Fly to the weed lat/lon at cruise altitude before descending.
    Approach,
    /// Drone is overhead; descend to extract_alt_m using rangefinder when available.
    Descend,
    /// Hold at extraction altitude for WEED_STABILIZE_MS before actuating.
    Stabilize,
    /// Servo deployed — hold position and altitude for WEED_PULL_MS.
    Extract,
    /// Grip check failed — jaw reopened; hold for GRIP_RELEASE_MS, then
    /// re-stabilise and try the grab again (up to MAX_GRIP_ATTEMPTS).
    GripRelease,
    /// Climb back to cruise altitude (gripping weed, or empty after an abort —
    /// see `grip_confirmed`).
    Ascend,
    /// At cruise altitude — fly to home (bin receptacle) position.
    Dispose,
    /// Over the bin — descend to BIN_DROP_ALT_M and release.
    DisposeDescend,
}

// ── Constants ─────────────────────────────────────────────────────────────────

pub const MAX_WAYPOINTS: usize = 32;

const WAYPOINT_RADIUS_M: f32 = 2.0;
const TAKEOFF_ALT_M:     f32 = 2.0;          // auto-takeoff climb height (baro, AGL)
const TAKEOFF_BAND_M:    f32 = 0.3;          // "reached takeoff altitude" tolerance
const GEOFENCE_MAX_ALT_M:    f32 = 120.0;    // altitude ceiling, AGL (~400 ft)
const GEOFENCE_MAX_RADIUS_M: f32 = 300.0;    // max horizontal distance from home
const LAND_DESCENT_MPS:  f32 = 0.5;          // m/s target descent rate
const BATT_LOW_PCT:      u8  = 20;           // trigger RTH when SoC drops below this
const CRUISE_SPEED_MPS:  f32 = 5.0;          // nominal cruise (sets error cap)
const NAV_ERR_CAP_M:     f32 = CRUISE_SPEED_MPS * 4.0; // 20 m — clamps PID input on long legs
const MAX_TILT_RAD:      f32 = 0.436;         // 25°
const RTH_MIN_ALT_M:     f32 = 15.0;         // RTH cruise altitude floor
const HOME_ARRIVE_M:     f32 = 5.0;          // switch RTH → descend within this radius
const YAW_TRACK_KP:      f32 = 0.8;          // rad/s per rad heading error
const FLOW_DAMP_KP:       f32 = 0.3;          // rad/(m/s) — velocity damping gain in PositionHold
const FLOW_MAX_HEIGHT_MM: i32 = 5_000;        // MTF-02P reliable range ceiling (5 m)
const DT:                 f32 = 1.0 / 100.0;

// Weed extraction sequence
const WEED_ARRIVE_M:      f32  = 1.5;         // horizontal arrival radius (GPS-realistic)
const WEED_ALT_BAND_M:    f32  = 0.15;        // "at extraction altitude" tolerance
const WEED_STABILIZE_MS:  u64  = 1_000;       // hover at extraction alt before actuating
const WEED_PULL_MS:       u64  = 500;         // servo hold duration
// Grip-confirmation (`--features grip-sense`, microswitch on PC2→GND, closed = held):
const MAX_GRIP_ATTEMPTS:  u8   = 3;           // grab tries before aborting this weed
const GRIP_RELEASE_MS:    u64  = 400;         // jaw-reopen dwell before a retry
const WEED_ASCEND_NEAR_M: f32  = 1.0;         // within this of approach alt → ascent complete
const BIN_DROP_ALT_M:     f32  = 0.5;         // AGL to descend to over bin before releasing

// ── Public types ──────────────────────────────────────────────────────────────

#[derive(Clone, Copy, Default)]
pub struct Waypoint {
    pub position:    LatLonAlt,
    pub hold_time_s: f32, // hover at waypoint before advancing (0 = fly-through)
}

/// Upload buffer written by telemetry_task via MAVLink mission protocol.
/// navigation_task loads it when `ready` is set and clears the flag.
pub struct MissionUpload {
    pub waypoints: [Waypoint; MAX_WAYPOINTS],
    pub count:     usize,
    pub ready:     bool,
}

impl MissionUpload {
    pub const fn new() -> Self {
        Self {
            waypoints: [Waypoint {
                position:    LatLonAlt { lat_deg: 0.0, lon_deg: 0.0, alt_m: 0.0 },
                hold_time_s: 0.0,
            }; MAX_WAYPOINTS],
            count: 0,
            ready: false,
        }
    }
}

pub static PENDING_MISSION: Mutex<CriticalSectionRawMutex, MissionUpload> =
    Mutex::new(MissionUpload::new());

// ── Internal mission sequencer ────────────────────────────────────────────────

struct Mission {
    waypoints: [Waypoint; MAX_WAYPOINTS],
    count:     usize,
    current:   usize,
    home:      LatLonAlt,
}

impl Mission {
    const fn new() -> Self {
        Self {
            waypoints: [Waypoint {
                position:    LatLonAlt { lat_deg: 0.0, lon_deg: 0.0, alt_m: 0.0 },
                hold_time_s: 0.0,
            }; MAX_WAYPOINTS],
            count:   0,
            current: 0,
            home:    LatLonAlt { lat_deg: 0.0, lon_deg: 0.0, alt_m: 0.0 },
        }
    }

    fn load(&mut self, up: &MissionUpload) {
        let n = up.count.min(MAX_WAYPOINTS);
        self.waypoints[..n].copy_from_slice(&up.waypoints[..n]);
        self.count   = n;
        self.current = 0;
    }

    fn current_wp(&self) -> Option<Waypoint> {
        if self.current < self.count { Some(self.waypoints[self.current]) } else { None }
    }

    fn advance(&mut self) { self.current = (self.current + 1).min(self.count); }
    fn is_complete(&self) -> bool { self.current >= self.count }
}

// ── Geo helpers ───────────────────────────────────────────────────────────────

pub fn haversine_m(a: LatLonAlt, b: LatLonAlt) -> f32 {
    const R: f32 = 6_371_000.0;
    let lat1 = (a.lat_deg as f32).to_radians();
    let lat2 = (b.lat_deg as f32).to_radians();
    let dlat = ((b.lat_deg - a.lat_deg) as f32).to_radians();
    let dlon = ((b.lon_deg - a.lon_deg) as f32).to_radians();
    let sl = sinf(dlat / 2.0);
    let sd = sinf(dlon / 2.0);
    2.0 * R * libm::asinf(sqrtf((sl*sl + cosf(lat1)*cosf(lat2)*sd*sd).min(1.0)))
}

fn wrap_pi(a: f32) -> f32 {
    const PI:  f32 = core::f32::consts::PI;
    const TAU: f32 = core::f32::consts::TAU;
    let a = a % TAU;
    if a > PI { a - TAU } else if a < -PI { a + TAU } else { a }
}

// ── Guidance ──────────────────────────────────────────────────────────────────

/// Fly toward `target` using NED position PIDs + altitude PID.
/// Yaws toward the target bearing when more than 3 m away.
fn guide_to(
    target:      LatLonAlt,
    pos:         LatLonAlt,
    target_alt:  f32,
    current_alt: f32,
    yaw:         f32,
    pid_n:       &mut PosPid,
    pid_e:       &mut PosPid,
    alt_pid:     &mut AltPid,
) -> NavCommand {
    let lat_rad = (pos.lat_deg as f32).to_radians();
    let raw_n = ((target.lat_deg - pos.lat_deg) * 111_320.0) as f32;
    let raw_e = ((target.lon_deg - pos.lon_deg) * 111_320.0
                 * cosf(lat_rad) as f64) as f32;

    let err_n = raw_n.clamp(-NAV_ERR_CAP_M, NAV_ERR_CAP_M);
    let err_e = raw_e.clamp(-NAV_ERR_CAP_M, NAV_ERR_CAP_M);

    let fwd_m   =  err_n * cosf(yaw) + err_e * sinf(yaw);
    let right_m = -err_n * sinf(yaw) + err_e * cosf(yaw);

    let dist     = sqrtf(raw_n * raw_n + raw_e * raw_e);
    let bearing  = atan2f(raw_e, raw_n); // NED: 0=North, +90°=East
    let yaw_rate = if dist > 3.0 {
        (YAW_TRACK_KP * wrap_pi(bearing - yaw)).clamp(-1.0, 1.0)
    } else {
        0.0
    };

    NavCommand {
        autonomous: true,
        attitude_setpoint: AttitudeSetpoint {
            roll:     pid_e.update(right_m).clamp(-MAX_TILT_RAD, MAX_TILT_RAD),
            pitch:    (-pid_n.update(fwd_m)).clamp(-MAX_TILT_RAD, MAX_TILT_RAD),
            yaw_rate,
            throttle: alt_pid.update(target_alt, current_alt),
        },
        target,
    }
}

/// Controlled descent at LAND_DESCENT_MPS using a moving target altitude.
/// Switches to rangefinder AGL for the final 2 m; cuts throttle at touch-down.
fn land_step(target: &mut f32, alt_pid: &mut AltPid, baro_alt: f32, flow: &FlowData) -> NavCommand {
    let agl = if flow.valid && flow.height_mm > 0 && flow.height_mm < 2000 {
        flow.height_mm as f32 / 1000.0
    } else {
        baro_alt
    };

    if agl < 0.05 {
        return NavCommand {
            autonomous: true,
            attitude_setpoint: AttitudeSetpoint { throttle: 0.0, roll: 0.0, pitch: 0.0, yaw_rate: 0.0 },
            target: Default::default(),
        };
    }

    *target = (*target - LAND_DESCENT_MPS * DT).max(0.0);

    NavCommand {
        autonomous: true,
        attitude_setpoint: AttitudeSetpoint {
            roll: 0.0, pitch: 0.0, yaw_rate: 0.0,
            throttle: alt_pid.update(*target, baro_alt),
        },
        target: Default::default(),
    }
}

// ── Embassy task ──────────────────────────────────────────────────────────────

/// `grip_pin`: gripper-jaw microswitch (PC2 → GND, internal pull-up; closed = weed
/// held = LOW). Only consulted when built with `--features grip-sense` — without the
/// feature the Extract phase trusts the WEED_PULL_MS timer as before, so benches and
/// airframes without the switch fly unchanged.
#[embassy_executor::task]
pub async fn navigation_task(grip_pin: embassy_stm32::gpio::Input<'static>) {
    let mut mission = Mission::new();

    // Altitude PID — shared across AltHold, PosHold, Auto, RTH, Land.
    // Reset on every mode change so the I-term doesn't carry stale state.
    let mut alt_pid   = AltPid::new();

    // Horizontal PIDs for PositionHold (separate from nav so tuning can differ)
    let mut pos_pid_n = PosPid::new();
    let mut pos_pid_e = PosPid::new();

    // Horizontal PIDs for Auto + RTH
    let mut nav_pid_n = PosPid::new();
    let mut nav_pid_e = PosPid::new();

    let mut hold_alt:         f32                  = 0.0;
    let mut hold_pos:         LatLonAlt            = Default::default();
    let mut rth_cruise_alt:   f32                  = 0.0;
    let mut land_target:      f32                  = 0.0;
    let mut rth_landing:      bool                 = false;
    let mut wp_arrived_at:    Option<Instant>      = None;
    let mut weed_phase:       Option<WeedPhase>    = None;
    let mut weed_approach_alt: f32                 = 0.0;
    let mut weed_phase_timer: Option<Instant>      = None;
    let mut grip_attempts:    u8                   = 0;
    let mut grip_confirmed:   bool                 = false;
    let mut prev_mode:        FlightMode           = FlightMode::Stabilise;
    let mut home_set:         bool                 = false;
    let mut last_gps_ok:      Instant              = Instant::now();
    let mut prev_armed:       bool                 = false;
    let mut takeoff_done:     bool                 = false;
    let mut takeoff_target_alt: f32               = 0.0;

    info!("Navigation task started (100 Hz)");
    let mut ticker = Ticker::every(Duration::from_hz(100));

    loop {
        ticker.next().await;

        let mode     = STATE.rc_input.lock().await.mode;
        let gps      = *STATE.gps_fix.lock().await;
        let baro     = *STATE.baro_data.lock().await;
        let attitude = *STATE.attitude.lock().await;
        let battery  = *STATE.battery.lock().await;
        let is_armed = *STATE.armed.lock().await;
        let flow     = *STATE.flow.lock().await;

        if gps.fix_ok { last_gps_ok = Instant::now(); }

        let pos = gps.pos();
        let yaw = atan2f(
            2.0 * (attitude.w * attitude.z + attitude.x * attitude.y),
            1.0 - 2.0 * (attitude.y * attitude.y + attitude.z * attitude.z),
        );

        // Auto-takeoff bookkeeping: on the disarmed→armed edge, capture the takeoff
        // target (current baro + climb height) and require a fresh climb next mission.
        if is_armed && !prev_armed {
            takeoff_target_alt = baro.altitude_m + TAKEOFF_ALT_M;
            takeoff_done = false;
        }
        prev_armed = is_armed;

        if !is_armed {
            STATE.nav_command.lock().await.autonomous = false;
            continue;
        }

        // Capture home on first confirmed 3-D fix
        if !home_set && gps.fix_ok {
            mission.home = pos;
            home_set = true;
            info!("Home set: lat={=f64} lon={=f64}", gps.lat_deg, gps.lon_deg);
        }

        // Apply a DO_SET_HOME override from the GCS (MAV_CMD_DO_SET_HOME / cmd 179).
        // Overwrites the auto-captured home, allowing the operator to reset RTH origin mid-flight.
        if let Some(new_home) = STATE.home_override.lock().await.take() {
            mission.home = new_home;
            home_set = true;
            info!("Home overridden by GCS: lat={=f64} lon={=f64}", new_home.lat_deg, new_home.lon_deg);
        }

        // Fault state: force Land; auto-disarm at touch-down
        if state::get() == state::FlightState::Fault {
            let agl = if flow.valid && flow.height_mm > 0 && flow.height_mm <= FLOW_MAX_HEIGHT_MM {
                flow.height_mm as f32 / 1000.0
            } else {
                baro.altitude_m
            };
            if agl < 0.15 {
                *STATE.armed.lock().await = false;
                info!("Fault landing complete — disarmed");
            }
        }

        // Resolve effective mode: Fault → Land; critical battery → RTH or Land
        let effective_mode = if state::get() == state::FlightState::Fault {
            FlightMode::Land
        } else if home_set
            && (baro.altitude_m > GEOFENCE_MAX_ALT_M
                || (gps.fix_ok && haversine_m(pos, mission.home) > GEOFENCE_MAX_RADIUS_M))
            && !matches!(mode, FlightMode::Land | FlightMode::ReturnToHome)
        {
            // Geofence breach (altitude ceiling or distance from home) → come home / land.
            if gps.fix_ok {
                warn!("Geofence breach — forcing RTH");
                state::set(state::FlightState::Landing);
                FlightMode::ReturnToHome
            } else {
                warn!("Geofence breach — forcing Land");
                state::set(state::FlightState::Landing);
                FlightMode::Land
            }
        } else if battery.critical
            && !matches!(mode, FlightMode::Land | FlightMode::ReturnToHome)
        {
            if gps.fix_ok && home_set {
                warn!("Critical battery — forcing RTH");
                state::set(state::FlightState::Landing);
                FlightMode::ReturnToHome
            } else {
                warn!("Critical battery — forcing Land");
                state::set(state::FlightState::Landing);
                FlightMode::Land
            }
        } else if battery.voltage_v > 0.0
            && battery.pct < BATT_LOW_PCT
            && !battery.critical
            && !matches!(mode, FlightMode::Land | FlightMode::ReturnToHome)
        {
            if gps.fix_ok && home_set {
                warn!("Low battery ({}%) — forcing RTH", battery.pct);
                FlightMode::ReturnToHome
            } else {
                warn!("Low battery ({}%) — forcing Land (no GPS)", battery.pct);
                FlightMode::Land
            }
        } else if last_gps_ok.elapsed() > Duration::from_secs(5)
            && matches!(mode, FlightMode::PositionHold | FlightMode::Auto
                             | FlightMode::ReturnToHome | FlightMode::FollowMe)
        {
            warn!("GPS fix lost >5s — forcing AltitudeHold");
            FlightMode::AltitudeHold
        } else {
            mode
        };

        // Mode-entry initialisation — runs on the first tick of each new mode
        if effective_mode != prev_mode {
            alt_pid.reset();
            nav_pid_n.reset(); nav_pid_e.reset();
            rth_landing      = false;
            wp_arrived_at    = None;
            weed_phase_timer = None;
            if weed_phase.take().is_some() {
                STATE.servo_outputs.lock().await.s1 = 0.0;
                STATE.weed_target.lock().await.valid = false;
            }
            // Clear any stale follow/weed target when leaving Follow Me so Auto
            // mode doesn't immediately fly to the last person position.
            if prev_mode == FlightMode::FollowMe {
                STATE.weed_target.lock().await.valid = false;
            }

            match effective_mode {
                FlightMode::AltitudeHold => {
                    hold_alt = baro.altitude_m;
                    info!("AltHold: target={=f32}m", hold_alt);
                }
                FlightMode::PositionHold => {
                    hold_alt = baro.altitude_m;
                    hold_pos = pos;
                    pos_pid_n.reset(); pos_pid_e.reset();
                    info!("PosHold: alt={=f32}m lat={=f64} lon={=f64}",
                          hold_alt, pos.lat_deg, pos.lon_deg);
                }
                FlightMode::ReturnToHome => {
                    rth_cruise_alt = baro.altitude_m.max(RTH_MIN_ALT_M);
                    info!("RTH: climbing to {=f32}m then homing", rth_cruise_alt);
                }
                FlightMode::Land => {
                    land_target = baro.altitude_m;
                }
                FlightMode::FollowMe => {
                    hold_alt = baro.altitude_m;
                    hold_pos = pos;
                    STATE.weed_target.lock().await.valid = false;
                    info!("FollowMe: holding {=f32}m, fly to desired height first", hold_alt);
                }
                _ => {}
            }
        }

        let cmd = match effective_mode {
            // ── Stabilise: RC direct, no autonomous override ───────────────────
            FlightMode::Stabilise => {
                NavCommand { autonomous: false, ..Default::default() }
            }

            // ── AltitudeHold: lock altitude, pass RC roll/pitch/yaw ────────────
            FlightMode::AltitudeHold => {
                let rc = *STATE.rc_input.lock().await;
                let mut sp = rc.to_attitude_setpoint();
                sp.throttle = alt_pid.update(hold_alt, baro.altitude_m);
                NavCommand { autonomous: true, attitude_setpoint: sp, target: Default::default() }
            }

            // ── PositionHold: lock GPS + baro, RC yaw pass-through ────────────
            FlightMode::PositionHold => {
                let rc = *STATE.rc_input.lock().await;

                let (roll_sp, pitch_sp) = if gps.fix_ok {
                    let lat_rad = (gps.lat_deg as f32).to_radians();
                    let err_n   = ((hold_pos.lat_deg - gps.lat_deg) * 111_320.0) as f32;
                    let err_e   = ((hold_pos.lon_deg - gps.lon_deg) * 111_320.0
                                   * cosf(lat_rad) as f64) as f32;
                    let fwd_m   =  err_n * cosf(yaw) + err_e * sinf(yaw);
                    let right_m = -err_n * sinf(yaw) + err_e * cosf(yaw);
                    let mut pitch = -pos_pid_n.update(fwd_m);
                    let mut roll  =  pos_pid_e.update(right_m);

                    // Optical-flow velocity damping: oppose measured body-frame drift.
                    // vel_x/y are body-frame drone velocity (positive = forward/right).
                    // Polarity may need sign reversal depending on sensor mounting orientation.
                    if flow.valid && flow.height_mm > 0 && flow.height_mm <= FLOW_MAX_HEIGHT_MM {
                        let h_m          = flow.height_mm as f32 / 1000.0;
                        let vel_fwd_ms   = flow.vel_x_mrad_s as f32 * h_m / 1_000_000.0;
                        let vel_right_ms = flow.vel_y_mrad_s as f32 * h_m / 1_000_000.0;
                        pitch += FLOW_DAMP_KP * vel_fwd_ms;
                        roll  -= FLOW_DAMP_KP * vel_right_ms;
                    } else {
                        // ── Fused-estimate velocity damping (ADDED: complementary estimator) ──
                        // When optical flow is unavailable (too high / no rangefinder),
                        // damp drift using the fused NED velocity from estimator_task,
                        // rotated into body frame. This is purely additive damping on top
                        // of the unchanged GPS-direct position loop above; it never runs
                        // when flow is valid, so existing PosHold behaviour is preserved.
                        // REVERSIBLE: delete this entire `else` block to restore the
                        // original flow-only behaviour.
                        let est = *STATE.pos_estimate.lock().await;
                        if est.valid {
                            let vel_fwd_ms   =  est.vel_n * cosf(yaw) + est.vel_e * sinf(yaw);
                            let vel_right_ms = -est.vel_n * sinf(yaw) + est.vel_e * cosf(yaw);
                            pitch += FLOW_DAMP_KP * vel_fwd_ms;
                            roll  -= FLOW_DAMP_KP * vel_right_ms;
                        }
                    }

                    (roll.clamp(-MAX_TILT_RAD, MAX_TILT_RAD), pitch.clamp(-MAX_TILT_RAD, MAX_TILT_RAD))
                } else {
                    (0.0, 0.0)
                };

                NavCommand {
                    autonomous: true,
                    attitude_setpoint: AttitudeSetpoint {
                        roll:     roll_sp,
                        pitch:    pitch_sp,
                        yaw_rate: rc.yaw * core::f32::consts::PI, // RC yaw pass-through
                        throttle: alt_pid.update(hold_alt, baro.altitude_m),
                    },
                    target: hold_pos,
                }
            }

            // ── Auto: fly MAVLink mission waypoints; weed targets take priority ──
            FlightMode::Auto => {
                // Load any freshly uploaded mission
                {
                    let mut pm = PENDING_MISSION.lock().await;
                    if pm.ready {
                        mission.load(&*pm);
                        pm.ready = false;
                        nav_pid_n.reset(); nav_pid_e.reset();
                        wp_arrived_at = None;
                        info!("Mission loaded: {} waypoints", mission.count);
                    }
                }

                if !gps.fix_ok {
                    warn!("Auto: no GPS fix");
                    NavCommand { autonomous: false, ..Default::default() }
                } else if !takeoff_done {
                    // ── AUTO-TAKEOFF: climb to TAKEOFF_ALT_M before running the mission.
                    // Level attitude + alt PID to the captured target. NOTE: autonomous
                    // flight still requires an RC link up — control_task zeros motors on
                    // RC failsafe, so this assumes a safety-pilot link is present.
                    let climb = NavCommand {
                        autonomous: true,
                        attitude_setpoint: AttitudeSetpoint {
                            roll: 0.0, pitch: 0.0, yaw_rate: 0.0,
                            throttle: alt_pid.update(takeoff_target_alt, baro.altitude_m),
                        },
                        target: Default::default(),
                    };
                    if baro.altitude_m >= takeoff_target_alt - TAKEOFF_BAND_M {
                        takeoff_done = true;
                        nav_pid_n.reset();
                        nav_pid_e.reset();
                        info!("Auto: takeoff complete at {=f32} m — starting mission", baro.altitude_m);
                    }
                    climb
                } else {
                    let weed = *STATE.weed_target.lock().await;

                    // Use rangefinder AGL when within reliable range; fall back to baro.
                    let agl = if flow.valid && flow.height_mm > 0
                                && flow.height_mm <= FLOW_MAX_HEIGHT_MM {
                        flow.height_mm as f32 / 1000.0
                    } else {
                        baro.altitude_m
                    };

                    // Transition: new target arrived → start Approach phase.
                    if weed.valid && weed_phase.is_none() {
                        weed_approach_alt = baro.altitude_m;
                        weed_phase        = Some(WeedPhase::Approach);
                        weed_phase_timer  = None;
                        grip_attempts     = 0;
                        grip_confirmed    = false;
                        info!("Weed sequence start — approach alt {=f32}m", weed_approach_alt);
                    }

                    if let Some(phase) = weed_phase {
                        let cmd = match phase {
                            // ── Phase 1: fly to weed lat/lon at cruise altitude ──
                            WeedPhase::Approach => {
                                let dist = haversine_m(pos, weed.position);
                                if dist < WEED_ARRIVE_M {
                                    alt_pid.reset(); // altitude reference switches baro→AGL
                                    weed_phase       = Some(WeedPhase::Descend);
                                    weed_phase_timer = None;
                                    info!("Weed overhead (dist={=f32}m) — descending to {=f32}m AGL",
                                          dist, weed.extract_alt_m);
                                }
                                guide_to(weed.position, pos, weed_approach_alt, baro.altitude_m,
                                         yaw, &mut nav_pid_n, &mut nav_pid_e, &mut alt_pid)
                            }

                            // ── Phase 2: descend to extraction altitude ──────────
                            WeedPhase::Descend => {
                                if agl < weed.extract_alt_m + WEED_ALT_BAND_M {
                                    weed_phase       = Some(WeedPhase::Stabilize);
                                    weed_phase_timer = Some(Instant::now());
                                    info!("Extraction altitude reached ({=f32}m) — stabilising",
                                          agl);
                                }
                                guide_to(weed.position, pos, weed.extract_alt_m, agl,
                                         yaw, &mut nav_pid_n, &mut nav_pid_e, &mut alt_pid)
                            }

                            // ── Phase 3: hold for oscillations to damp out ───────
                            WeedPhase::Stabilize => {
                                let elapsed = weed_phase_timer
                                    .map_or(0, |t| t.elapsed().as_millis() as u64);
                                if elapsed >= WEED_STABILIZE_MS {
                                    STATE.servo_outputs.lock().await.s1 = 1.0;
                                    weed_phase       = Some(WeedPhase::Extract);
                                    weed_phase_timer = Some(Instant::now());
                                    info!("Stabilised — servo deployed");
                                }
                                guide_to(weed.position, pos, weed.extract_alt_m, agl,
                                         yaw, &mut nav_pid_n, &mut nav_pid_e, &mut alt_pid)
                            }

                            // ── Phase 4: servo deployed, hold for pull duration,
                            //    then confirm the grab actually caught something ──
                            WeedPhase::Extract => {
                                let elapsed = weed_phase_timer
                                    .map_or(0, |t| t.elapsed().as_millis() as u64);
                                if elapsed >= WEED_PULL_MS {
                                    // Grip check: jaw microswitch closes to GND when
                                    // something is held (LOW = held). Without the
                                    // grip-sense feature, trust the timer (legacy
                                    // open-loop behaviour — no switch fitted).
                                    #[cfg(feature = "grip-sense")]
                                    let held = grip_pin.is_low();
                                    #[cfg(not(feature = "grip-sense"))]
                                    let held = { let _ = &grip_pin; true };

                                    if held {
                                        // Keep s1 = 1.0 — maintain grip while ascending.
                                        grip_confirmed   = true;
                                        weed_phase       = Some(WeedPhase::Ascend);
                                        weed_phase_timer = None;
                                        info!("Weed grabbed — ascending to {=f32}m with payload",
                                              weed_approach_alt);
                                    } else {
                                        grip_attempts = grip_attempts.saturating_add(1);
                                        STATE.servo_outputs.lock().await.s1 = 0.0;
                                        if grip_attempts < MAX_GRIP_ATTEMPTS {
                                            weed_phase       = Some(WeedPhase::GripRelease);
                                            weed_phase_timer = Some(Instant::now());
                                            defmt::warn!("Grip check FAILED (attempt {}/{}) — reopening for retry",
                                                  grip_attempts, MAX_GRIP_ATTEMPTS);
                                        } else {
                                            // Out of attempts: climb away empty; the
                                            // Ascend branch aborts instead of flying
                                            // to the bin (grip_confirmed = false).
                                            grip_confirmed   = false;
                                            weed_phase       = Some(WeedPhase::Ascend);
                                            weed_phase_timer = None;
                                            defmt::warn!("Grip FAILED {} times — aborting this weed, ascending empty",
                                                  MAX_GRIP_ATTEMPTS);
                                        }
                                    }
                                }
                                guide_to(weed.position, pos, weed.extract_alt_m, agl,
                                         yaw, &mut nav_pid_n, &mut nav_pid_e, &mut alt_pid)
                            }

                            // ── Phase 4b: jaw reopened after a failed grab — dwell,
                            //    then re-stabilise and try again ──────────────────
                            WeedPhase::GripRelease => {
                                let elapsed = weed_phase_timer
                                    .map_or(0, |t| t.elapsed().as_millis() as u64);
                                if elapsed >= GRIP_RELEASE_MS {
                                    weed_phase       = Some(WeedPhase::Stabilize);
                                    weed_phase_timer = Some(Instant::now());
                                    info!("Jaw reopened — re-stabilising for grab attempt {}",
                                          grip_attempts + 1);
                                }
                                guide_to(weed.position, pos, weed.extract_alt_m, agl,
                                         yaw, &mut nav_pid_n, &mut nav_pid_e, &mut alt_pid)
                            }

                            // ── Phase 5: climb to cruise alt (gripping, or empty
                            //    after an aborted grab) ────────────────────────────
                            WeedPhase::Ascend => {
                                if agl > weed_approach_alt - WEED_ASCEND_NEAR_M {
                                    nav_pid_n.reset(); nav_pid_e.reset();
                                    if grip_confirmed {
                                        weed_phase       = Some(WeedPhase::Dispose);
                                        weed_phase_timer = None;
                                        info!("At cruise alt — flying home to drop weed");
                                    } else {
                                        // Nothing in the jaw — skip the bin run,
                                        // drop this target, resume the mission.
                                        STATE.weed_target.lock().await.valid = false;
                                        weed_phase       = None;
                                        weed_phase_timer = None;
                                        defmt::warn!("Ascended empty — weed target dropped, resuming mission");
                                    }
                                }
                                guide_to(weed.position, pos, weed_approach_alt, agl,
                                         yaw, &mut nav_pid_n, &mut nav_pid_e, &mut alt_pid)
                            }

                            // ── Phase 6: fly to home (bin) at cruise altitude ────
                            WeedPhase::Dispose => {
                                let dist_home = haversine_m(pos, mission.home);
                                if dist_home < WEED_ARRIVE_M {
                                    nav_pid_n.reset(); nav_pid_e.reset();
                                    weed_phase = Some(WeedPhase::DisposeDescend);
                                    info!("Over bin — descending to {=f32}m to drop",
                                          BIN_DROP_ALT_M);
                                }
                                guide_to(mission.home, pos, weed_approach_alt, agl,
                                         yaw, &mut nav_pid_n, &mut nav_pid_e, &mut alt_pid)
                            }

                            // ── Phase 7: descend over bin, release, done ──────────
                            WeedPhase::DisposeDescend => {
                                if agl < BIN_DROP_ALT_M + WEED_ALT_BAND_M {
                                    STATE.servo_outputs.lock().await.s1 = 0.0;
                                    STATE.weed_target.lock().await.valid = false;
                                    weed_phase       = None;
                                    weed_phase_timer = None;
                                    info!("Weed released into bin — resuming mission");
                                }
                                guide_to(mission.home, pos, BIN_DROP_ALT_M, agl,
                                         yaw, &mut nav_pid_n, &mut nav_pid_e, &mut alt_pid)
                            }
                        };
                        cmd
                    } else if mission.is_complete() {
                        // Hold current position once mission is done
                        guide_to(pos, pos, baro.altitude_m, baro.altitude_m,
                                 yaw, &mut nav_pid_n, &mut nav_pid_e, &mut alt_pid)
                    } else if let Some(wp) = mission.current_wp() {
                        let dist = haversine_m(pos, wp.position);

                        if dist < WAYPOINT_RADIUS_M {
                            if wp_arrived_at.is_none() {
                                wp_arrived_at = Some(Instant::now());
                                info!("WP {} reached (hold {=f32}s)", mission.current, wp.hold_time_s);
                            }
                            if wp_arrived_at.map_or(false, |t| {
                                t.elapsed() >= Duration::from_millis((wp.hold_time_s * 1000.0) as u64)
                            }) {
                                mission.advance();
                                wp_arrived_at = None;
                                info!("Advancing to WP {}", mission.current);
                            }
                        } else {
                            wp_arrived_at = None;
                        }

                        guide_to(wp.position, pos, wp.position.alt_m, baro.altitude_m,
                                 yaw, &mut nav_pid_n, &mut nav_pid_e, &mut alt_pid)
                    } else {
                        // Unreachable: !is_complete() guarantees current < count.
                        NavCommand { autonomous: false, ..Default::default() }
                    }
                }
            }

            // ── FollowMe: continuously track GCS person position at fixed alt ──
            FlightMode::FollowMe => {
                if !gps.fix_ok {
                    warn!("FollowMe: no GPS fix");
                    NavCommand { autonomous: false, ..Default::default() }
                } else {
                    // Consume the latest position target from the Pi.
                    // hold_alt (baro, captured at mode entry) is used for altitude
                    // so the drone stays at the operator's chosen height regardless
                    // of the GPS MSL value in the target message.
                    {
                        let mut wt = STATE.weed_target.lock().await;
                        if wt.valid {
                            hold_pos.lat_deg = wt.position.lat_deg;
                            hold_pos.lon_deg = wt.position.lon_deg;
                            wt.valid = false;
                        }
                    }
                    guide_to(hold_pos, pos, hold_alt, baro.altitude_m,
                             yaw, &mut nav_pid_n, &mut nav_pid_e, &mut alt_pid)
                }
            }

            // ── ReturnToHome: climb to safe alt, fly home, descend ────────────
            FlightMode::ReturnToHome => {
                if !home_set || !gps.fix_ok {
                    warn!("RTH: no home/GPS — landing");
                    if !rth_landing { land_target = baro.altitude_m; rth_landing = true; }
                    land_step(&mut land_target, &mut alt_pid, baro.altitude_m, &flow)
                } else {
                    let dist = haversine_m(pos, mission.home);
                    if rth_landing || dist < HOME_ARRIVE_M {
                        if !rth_landing {
                            land_target = baro.altitude_m;
                            rth_landing = true;
                            info!("RTH: home reached — descending");
                        }
                        land_step(&mut land_target, &mut alt_pid, baro.altitude_m, &flow)
                    } else {
                        guide_to(mission.home, pos, rth_cruise_alt, baro.altitude_m,
                                 yaw, &mut nav_pid_n, &mut nav_pid_e, &mut alt_pid)
                    }
                }
            }

            // ── Land: rate-controlled descent with rangefinder touch-down ──────
            FlightMode::Land => {
                land_step(&mut land_target, &mut alt_pid, baro.altitude_m, &flow)
            }
        };

        *STATE.nav_command.lock().await = cmd;
        prev_mode = effective_mode;
    }
}
