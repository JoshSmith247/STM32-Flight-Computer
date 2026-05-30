//! MAVLink v2 telemetry — USART3 full-duplex at 57600 baud.
//!
//! TX: 7 messages staggered across 10 Hz ticks (see reading/mavlink_design.md)
//!     + MISSION_REQUEST_INT / MISSION_ACK for mission upload handshake
//! RX: MAVLink v2 frame parser — HEARTBEAT watchdog, COMMAND_LONG dispatcher,
//!     MISSION_COUNT + MISSION_ITEM_INT for MAVLink mission protocol

use core::cell::Cell;

use defmt::{info, warn};
use embassy_futures::join::join;
use embassy_stm32::{
    peripherals,
    usart::{Config, Uart},
    Peri,
};
use embassy_time::{Duration, Instant, Ticker};

use crate::{
    navigation::{self, Waypoint, MAX_WAYPOINTS},
    state::{self, FlightState},
    types::{FlightMode, LatLonAlt},
    STATE,
};

const MAVLINK_SYSTEM_ID:     u8 = 1;
const MAVLINK_COMPONENT_ID:  u8 = 1;
const MAV_TYPE_QUAD:         u8 = 2;
const MAV_AUTOPILOT_GENERIC: u8 = 0;
const PI_HEARTBEAT_TIMEOUT:  Duration = Duration::from_secs(3);

// CRC_EXTRA (MAGIC) per message definition
const MAGIC_HEARTBEAT:           u8 = 50;
const MAGIC_SYS_STATUS:          u8 = 124;
const MAGIC_GPS_RAW_INT:         u8 = 24;
const MAGIC_ATTITUDE:            u8 = 39;
const MAGIC_SERVO_OUTPUT_RAW:    u8 = 222;
const MAGIC_VFR_HUD:             u8 = 20;
const MAGIC_COMMAND_LONG:        u8 = 152;
const MAGIC_COMMAND_ACK:         u8 = 143;
const MAGIC_BATTERY_STATUS:      u8 = 154;
const MAGIC_MISSION_COUNT:       u8 = 221;
const MAGIC_MISSION_ITEM_INT:    u8 = 38;
const MAGIC_MISSION_REQUEST_INT: u8 = 196;
const MAGIC_MISSION_ACK:         u8 = 153;

const MAV_RESULT_ACCEPTED:    u8 = 0;
const MAV_RESULT_UNSUPPORTED: u8 = 3;
const MAV_MISSION_ACCEPTED:   u8 = 0;

// ── Frame builder ─────────────────────────────────────────────────────────────

fn write_frame(buf: &mut [u8], seq: u8, msg_id: u32, payload: &[u8], crc_extra: u8) -> usize {
    let n = payload.len();
    buf[0] = 0xFD;
    buf[1] = n as u8;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = seq;
    buf[5] = MAVLINK_SYSTEM_ID;
    buf[6] = MAVLINK_COMPONENT_ID;
    buf[7] = (msg_id & 0xFF) as u8;
    buf[8] = ((msg_id >>  8) & 0xFF) as u8;
    buf[9] = ((msg_id >> 16) & 0xFF) as u8;
    buf[10..10 + n].copy_from_slice(payload);
    let crc = mavlink_crc(&buf[1..10 + n], crc_extra);
    buf[10 + n]     = (crc & 0xFF) as u8;
    buf[10 + n + 1] = (crc >>    8) as u8;
    12 + n
}

fn mavlink_crc(data: &[u8], extra: u8) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data.iter().chain(core::iter::once(&extra)) {
        let mut tmp = byte ^ (crc as u8);
        tmp ^= tmp << 4;
        crc = (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4);
    }
    crc
}

// ── Byte helpers ──────────────────────────────────────────────────────────────

fn f32_le(d: &[u8], off: usize) -> f32 {
    f32::from_le_bytes([d[off], d[off+1], d[off+2], d[off+3]])
}
fn u16_le(d: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([d[off], d[off+1]])
}
fn i32_le(d: &[u8], off: usize) -> i32 {
    i32::from_le_bytes([d[off], d[off+1], d[off+2], d[off+3]])
}

// ── Euler conversion ──────────────────────────────────────────────────────────

fn quat_to_euler(q: crate::types::Quaternion) -> (f32, f32, f32) {
    let roll  = libm::atan2f(2.0*(q.w*q.x + q.y*q.z), 1.0 - 2.0*(q.x*q.x + q.y*q.y));
    let pitch = libm::asinf((2.0*(q.w*q.y - q.z*q.x)).clamp(-1.0, 1.0));
    let yaw   = libm::atan2f(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
    (roll, pitch, yaw)
}

// ── TX payload builders ───────────────────────────────────────────────────────

// custom_mode layout (autopilot-defined u32):
//   bits  [7:0]  = FlightMode  (0=Stab … 5=Land)
//   bits [15:8]  = FlightState (0=Idle … 5=Fault)
//   bits [31:16] = payload_flags (bit 16 = servo bus, …)
fn build_heartbeat(flight_state: u8, mode: FlightMode, payload_flags: u32) -> [u8; 9] {
    let custom = (mode as u32)
        | ((flight_state as u32) << 8)
        | ((payload_flags & 0xFFFF) << 16);
    let mav_state = match flight_state {
        3 | 4 => 4, // Flying / Landing → MAV_STATE_ACTIVE
        5     => 5, // Fault            → MAV_STATE_CRITICAL
        _     => 3, // Idle / Arming / Armed → MAV_STATE_STANDBY
    };
    let is_armed = matches!(flight_state, 2 | 3 | 4);
    [
        (custom & 0xFF) as u8, ((custom >> 8) & 0xFF) as u8,
        ((custom >> 16) & 0xFF) as u8, ((custom >> 24) & 0xFF) as u8,
        MAV_TYPE_QUAD, MAV_AUTOPILOT_GENERIC,
        if is_armed { 0x80 } else { 0x00 },
        mav_state,
        2,
    ]
}

// SYS_STATUS #1 — 31 bytes: 3×u32, 9×u16/i16, 1×i8
fn build_sys_status(voltage_mv: u16, pct: u8) -> [u8; 31] {
    let mut p = [0u8; 31];
    let sensors: u32 = 0x61; // IMU 0x1, baro 0x20, GPS 0x40
    p[0..4].copy_from_slice(&sensors.to_le_bytes());
    p[4..8].copy_from_slice(&sensors.to_le_bytes());
    p[8..12].copy_from_slice(&sensors.to_le_bytes());
    p[14..16].copy_from_slice(&voltage_mv.to_le_bytes());
    p[16..18].copy_from_slice(&(-1i16).to_le_bytes()); // current unknown
    p[30] = pct;
    p
}

// GPS_RAW_INT #24 — 30 bytes: u64, 3×i32, 4×u16, 2×u8
fn build_gps_raw(gps: &crate::types::GpsFix, time_ms: u32) -> [u8; 30] {
    let mut p = [0u8; 30];
    let time_us: u64 = (time_ms as u64) * 1000;
    p[0..8].copy_from_slice(&time_us.to_le_bytes());
    p[8..12].copy_from_slice(&((gps.lat_deg * 1e7) as i32).to_le_bytes());
    p[12..16].copy_from_slice(&((gps.lon_deg * 1e7) as i32).to_le_bytes());
    p[16..20].copy_from_slice(&((gps.alt_m * 1000.0) as i32).to_le_bytes());
    let eph = ((gps.hacc_m * 100.0) as u32).min(65535) as u16;
    p[20..22].copy_from_slice(&eph.to_le_bytes());
    p[22..24].copy_from_slice(&u16::MAX.to_le_bytes());
    let spd = libm::sqrtf(gps.vel_n_ms*gps.vel_n_ms + gps.vel_e_ms*gps.vel_e_ms);
    p[24..26].copy_from_slice(&(((spd * 100.0) as u32).min(65535) as u16).to_le_bytes());
    p[26..28].copy_from_slice(&u16::MAX.to_le_bytes());
    p[28] = if gps.fix_ok { 3 } else { 1 };
    p[29] = 255;
    p
}

// ATTITUDE #30 — 28 bytes (all f32)
fn build_attitude(
    time_ms: u32, roll: f32, pitch: f32, yaw: f32,
    rs: f32, ps: f32, ys: f32,
) -> [u8; 28] {
    let mut p = [0u8; 28];
    p[0..4].copy_from_slice(&time_ms.to_le_bytes());
    p[4..8].copy_from_slice(&roll.to_bits().to_le_bytes());
    p[8..12].copy_from_slice(&pitch.to_bits().to_le_bytes());
    p[12..16].copy_from_slice(&yaw.to_bits().to_le_bytes());
    p[16..20].copy_from_slice(&rs.to_bits().to_le_bytes());
    p[20..24].copy_from_slice(&ps.to_bits().to_le_bytes());
    p[24..28].copy_from_slice(&ys.to_bits().to_le_bytes());
    p
}

// SERVO_OUTPUT_RAW #36 — 21 bytes: u32, 8×u16, u8
fn build_servo_output(time_ms: u32, motors: [f32; 4], servos: [f32; 4]) -> [u8; 21] {
    let mut p = [0u8; 21];
    p[0..4].copy_from_slice(&time_ms.to_le_bytes());
    let to_us = |v: f32| -> u16 { (1000.0 + v * 1000.0).clamp(1000.0, 2000.0) as u16 };
    for i in 0..4 { p[4 + i*2..6 + i*2].copy_from_slice(&to_us(motors[i]).to_le_bytes()); }
    for i in 0..4 { p[12 + i*2..14 + i*2].copy_from_slice(&to_us(servos[i]).to_le_bytes()); }
    p
}

// VFR_HUD #74 — 20 bytes: 4×f32, i16, u16
fn build_vfr_hud(baro_alt: f32, gnd_spd: f32, yaw_rad: f32, throttle: f32, climb: f32) -> [u8; 20] {
    let mut p = [0u8; 20];
    p[0..4].copy_from_slice(&0.0f32.to_bits().to_le_bytes()); // airspeed (no sensor)
    p[4..8].copy_from_slice(&gnd_spd.to_bits().to_le_bytes());
    p[8..12].copy_from_slice(&baro_alt.to_bits().to_le_bytes());
    p[12..16].copy_from_slice(&climb.to_bits().to_le_bytes());
    let hdg = ((yaw_rad * (180.0 / core::f32::consts::PI)) as i32).rem_euclid(360) as i16;
    p[16..18].copy_from_slice(&hdg.to_le_bytes());
    p[18..20].copy_from_slice(&((throttle * 100.0).clamp(0.0, 100.0) as u16).to_le_bytes());
    p
}

// BATTERY_STATUS #147 — 36 bytes: 2×i32, i16, 10×u16, i16, 3×u8, i8
fn build_battery_status(voltage_v: f32, pct: u8) -> [u8; 36] {
    let mut p = [0u8; 36];
    p[0..4].copy_from_slice(&(-1i32).to_le_bytes());
    p[4..8].copy_from_slice(&(-1i32).to_le_bytes());
    p[8..10].copy_from_slice(&i16::MAX.to_le_bytes()); // temperature unknown
    p[10..12].copy_from_slice(&((voltage_v / 4.0 * 1000.0) as u16).to_le_bytes());
    for i in 1..10usize { p[10 + i*2..12 + i*2].copy_from_slice(&u16::MAX.to_le_bytes()); }
    p[30..32].copy_from_slice(&(-1i16).to_le_bytes());
    p[34] = 3; // MAV_BATTERY_TYPE_LIPO
    p[35] = pct;
    p
}

// COMMAND_ACK #77 — 3 bytes (truncated; trailing extensions are zero)
fn build_command_ack(cmd: u16, result: u8) -> [u8; 3] {
    let mut p = [0u8; 3];
    p[0..2].copy_from_slice(&cmd.to_le_bytes());
    p[2] = result;
    p
}

// MISSION_REQUEST_INT #51 — 4 bytes: seq(u16), target_sys(u8), target_comp(u8)
fn build_mission_request_int(seq: u16) -> [u8; 4] {
    let mut p = [0u8; 4];
    p[0..2].copy_from_slice(&seq.to_le_bytes());
    p[2] = 255; // target GCS (broadcast)
    p[3] = 0;
    p
}

// MISSION_ACK #47 — 3 bytes: target_sys(u8), target_comp(u8), type(u8)
fn build_mission_ack(result: u8) -> [u8; 3] {
    [255, 0, result]
}

// ── Command dispatcher ────────────────────────────────────────────────────────

async fn handle_command(cmd: u16, param1: f32, param2: f32) -> u8 {
    match cmd {
        400 => {
            if param1 >= 0.5 {
                let rc = *STATE.rc_input.lock().await;
                if rc.throttle < 0.05 && !rc.failsafe && state::get() != FlightState::Fault {
                    *STATE.armed.lock().await = true;
                    state::set(FlightState::Flying);
                    info!("MAVLink: armed");
                    MAV_RESULT_ACCEPTED
                } else {
                    MAV_RESULT_UNSUPPORTED
                }
            } else {
                *STATE.armed.lock().await = false;
                state::set(FlightState::Idle);
                info!("MAVLink: disarmed");
                MAV_RESULT_ACCEPTED
            }
        }
        183 => {
            let idx  = param1 as u8;
            let norm = (param2.clamp(1000.0, 2000.0) - 1000.0) / 1000.0;
            let mut s = *STATE.servo_outputs.lock().await;
            match idx {
                1 => s.s1 = norm, 2 => s.s2 = norm,
                3 => s.s3 = norm, 4 => s.s4 = norm,
                _ => return MAV_RESULT_UNSUPPORTED,
            }
            *STATE.servo_outputs.lock().await = s;
            MAV_RESULT_ACCEPTED
        }
        20  => { STATE.rc_input.lock().await.mode = FlightMode::ReturnToHome; MAV_RESULT_ACCEPTED }
        21  => { STATE.rc_input.lock().await.mode = FlightMode::Land;         MAV_RESULT_ACCEPTED }
        176 => {
            let mode = match param2 as u8 {
                0 => FlightMode::Stabilise, 1 => FlightMode::AltitudeHold,
                2 => FlightMode::PositionHold, 3 => FlightMode::Auto,
                _ => return MAV_RESULT_UNSUPPORTED,
            };
            STATE.rc_input.lock().await.mode = mode;
            MAV_RESULT_ACCEPTED
        }
        _ => MAV_RESULT_UNSUPPORTED,
    }
}

// ── Embassy task ──────────────────────────────────────────────────────────────

#[embassy_executor::task]
pub async fn telemetry_task(
    uart_peri: Peri<'static, peripherals::USART3>,
    rx_pin:    Peri<'static, peripherals::PB11>,
    tx_pin:    Peri<'static, peripherals::PB10>,
    tx_dma:    Peri<'static, peripherals::DMA1_CH3>,
    rx_dma:    Peri<'static, peripherals::DMA1_CH1>,
    irqs:      crate::Irqs,
) {
    let mut config = Config::default();
    config.baudrate = 57600;
    let uart = Uart::new(uart_peri, rx_pin, tx_pin, tx_dma, rx_dma, irqs, config)
        .expect("USART3 init failed");
    let (mut tx, mut rx) = uart.split();

    info!("Telemetry: MAVLink v2 @ 57600 baud, 7-msg TX, mission protocol RX");

    // Inter-loop shared state (single-core cooperative — Cell is sufficient)
    let pi_ever_seen:        Cell<bool>              = Cell::new(false);
    let last_pi_hb:          Cell<Instant>           = Cell::new(Instant::now());
    let pending_ack:         Cell<Option<(u16, u8)>> = Cell::new(None);
    // Mission upload handshake
    let dl_count:            Cell<u16>               = Cell::new(0);
    let dl_next:             Cell<u16>               = Cell::new(0);
    let pending_mission_req: Cell<Option<u16>>        = Cell::new(None);
    let pending_mission_ack: Cell<Option<u8>>         = Cell::new(None);

    // ── TX loop ───────────────────────────────────────────────────────────────
    let tx_fut = async {
        let mut seq:     u8  = 0;
        let mut tick:    u8  = 0;
        let mut time_ms: u32 = 0;
        let mut buf = [0u8; 64];
        let mut ticker = Ticker::every(Duration::from_hz(10));

        loop {
            ticker.next().await;
            time_ms = time_ms.wrapping_add(100);

            // Pi heartbeat watchdog
            if pi_ever_seen.get() && last_pi_hb.get().elapsed() > PI_HEARTBEAT_TIMEOUT {
                if *STATE.armed.lock().await && state::get() != FlightState::Fault {
                    warn!("Pi heartbeat lost — forcing Fault");
                    state::set(FlightState::Fault);
                }
            }

            let quat = *STATE.attitude.lock().await;
            let imu  = *STATE.imu_data.lock().await;
            let (roll, pitch, yaw) = quat_to_euler(quat);

            // Every tick — ATTITUDE #30 @ 10 Hz
            {
                let p = build_attitude(time_ms, roll, pitch, yaw,
                                       imu.gyro.x, imu.gyro.y, imu.gyro.z);
                let n = write_frame(&mut buf, seq, 30, &p, MAGIC_ATTITUDE);
                tx.write(&buf[..n]).await.ok();
                seq = seq.wrapping_add(1);
            }

            // Even ticks — VFR_HUD #74 + SERVO_OUTPUT_RAW #36 @ 5 Hz
            if tick % 2 == 0 {
                let baro   = *STATE.baro_data.lock().await;
                let rc     = *STATE.rc_input.lock().await;
                let gps    = *STATE.gps_fix.lock().await;
                let motors = *STATE.motor_outputs.lock().await;
                let servos = *STATE.servo_outputs.lock().await;

                let gnd_spd = libm::sqrtf(gps.vel_n_ms*gps.vel_n_ms + gps.vel_e_ms*gps.vel_e_ms);
                let p = build_vfr_hud(baro.altitude_m, gnd_spd, yaw, rc.throttle, -gps.vel_d_ms);
                let n = write_frame(&mut buf, seq, 74, &p, MAGIC_VFR_HUD);
                tx.write(&buf[..n]).await.ok();
                seq = seq.wrapping_add(1);

                let p = build_servo_output(time_ms,
                    [motors.m1, motors.m2, motors.m3, motors.m4],
                    [servos.s1, servos.s2, servos.s3, servos.s4]);
                let n = write_frame(&mut buf, seq, 36, &p, MAGIC_SERVO_OUTPUT_RAW);
                tx.write(&buf[..n]).await.ok();
                seq = seq.wrapping_add(1);
            }

            // Staggered 1 Hz messages
            match tick % 10 {
                0 => {
                    let mode         = STATE.rc_input.lock().await.mode;
                    let flight_state = state::get() as u8;
                    let pl_flags     = *STATE.payload_flags.lock().await;
                    let p = build_heartbeat(flight_state, mode, pl_flags);
                    let n = write_frame(&mut buf, seq, 0, &p, MAGIC_HEARTBEAT);
                    tx.write(&buf[..n]).await.ok();
                    seq = seq.wrapping_add(1);
                    info!("HB tx — state={} mode={} payloads=0x{:04X}", flight_state, mode, pl_flags);
                }
                2 => {
                    let bat = *STATE.battery.lock().await;
                    let p = build_sys_status((bat.voltage_v * 1000.0) as u16, bat.pct);
                    let n = write_frame(&mut buf, seq, 1, &p, MAGIC_SYS_STATUS);
                    tx.write(&buf[..n]).await.ok();
                    seq = seq.wrapping_add(1);
                }
                4 => {
                    let gps = *STATE.gps_fix.lock().await;
                    let p = build_gps_raw(&gps, time_ms);
                    let n = write_frame(&mut buf, seq, 24, &p, MAGIC_GPS_RAW_INT);
                    tx.write(&buf[..n]).await.ok();
                    seq = seq.wrapping_add(1);
                }
                6 => {
                    let bat = *STATE.battery.lock().await;
                    let p = build_battery_status(bat.voltage_v, bat.pct);
                    let n = write_frame(&mut buf, seq, 147, &p, MAGIC_BATTERY_STATUS);
                    tx.write(&buf[..n]).await.ok();
                    seq = seq.wrapping_add(1);
                }
                _ => {}
            }

            // Send queued command ACK
            if let Some((cmd, result)) = pending_ack.get() {
                pending_ack.set(None);
                let p = build_command_ack(cmd, result);
                let n = write_frame(&mut buf, seq, 77, &p, MAGIC_COMMAND_ACK);
                tx.write(&buf[..n]).await.ok();
                seq = seq.wrapping_add(1);
            }

            // Mission upload handshake: request next item
            if let Some(req_seq) = pending_mission_req.get() {
                pending_mission_req.set(None);
                let p = build_mission_request_int(req_seq);
                let n = write_frame(&mut buf, seq, 51, &p, MAGIC_MISSION_REQUEST_INT);
                tx.write(&buf[..n]).await.ok();
                seq = seq.wrapping_add(1);
            }

            // Mission upload handshake: final ACK
            if let Some(ack_type) = pending_mission_ack.get() {
                pending_mission_ack.set(None);
                let p = build_mission_ack(ack_type);
                let n = write_frame(&mut buf, seq, 47, &p, MAGIC_MISSION_ACK);
                tx.write(&buf[..n]).await.ok();
                seq = seq.wrapping_add(1);
            }

            tick = tick.wrapping_add(1);
        }
    };

    // ── RX loop ───────────────────────────────────────────────────────────────
    let rx_fut = async {
        let mut sm:      u8        = 0; // 0=SYNC 1=HDR 2=PAY 3=CRC
        let mut hdr      = [0u8; 9];
        let mut hdr_idx: usize     = 0;
        let mut payload  = [0u8; 64];
        let mut pay_idx: usize     = 0;
        let mut crc_buf  = [0u8; 2];
        let mut crc_idx: usize     = 0;
        let mut byte     = [0u8; 1];

        loop {
            if rx.read(&mut byte).await.is_err() {
                sm = 0;
                continue;
            }
            let b = byte[0];

            match sm {
                0 => { if b == 0xFD { sm = 1; hdr_idx = 0; } }
                1 => {
                    hdr[hdr_idx] = b;
                    hdr_idx += 1;
                    if hdr_idx == 9 {
                        let len = hdr[0] as usize;
                        if len > 64 { sm = 0; }
                        else { pay_idx = 0; crc_idx = 0; sm = if len == 0 { 3 } else { 2 }; }
                    }
                }
                2 => {
                    payload[pay_idx] = b;
                    pay_idx += 1;
                    if pay_idx == hdr[0] as usize { crc_idx = 0; sm = 3; }
                }
                3 => {
                    crc_buf[crc_idx] = b;
                    crc_idx += 1;
                    if crc_idx < 2 { continue; }
                    sm = 0;

                    let pay_len = hdr[0] as usize;
                    let msg_id: u32 = (hdr[6] as u32)
                        | ((hdr[7] as u32) << 8) | ((hdr[8] as u32) << 16);

                    let extra = match msg_id {
                        0  => MAGIC_HEARTBEAT,
                        44 => MAGIC_MISSION_COUNT,
                        73 => MAGIC_MISSION_ITEM_INT,
                        76 => MAGIC_COMMAND_LONG,
                        _  => continue,
                    };

                    // Verify CRC over hdr[0..9] ++ payload[0..pay_len] ++ extra
                    let computed = {
                        let mut crc: u16 = 0xFFFF;
                        for &by in hdr.iter()
                            .chain(payload[..pay_len].iter())
                            .chain(core::iter::once(&extra))
                        {
                            let mut tmp = by ^ (crc as u8);
                            tmp ^= tmp << 4;
                            crc = (crc >> 8)
                                ^ ((tmp as u16) << 8)
                                ^ ((tmp as u16) << 3)
                                ^ ((tmp as u16) >> 4);
                        }
                        crc
                    };
                    if computed != u16::from_le_bytes(crc_buf) {
                        warn!("MAVLink CRC fail msgid={}", msg_id);
                        continue;
                    }

                    match msg_id {
                        0 => {
                            // HEARTBEAT — update Pi watchdog
                            if !pi_ever_seen.get() {
                                info!("Pi connected (USART3)");
                                pi_ever_seen.set(true);
                            }
                            last_pi_hb.set(Instant::now());
                        }

                        44 => {
                            // MISSION_COUNT — begin upload handshake
                            // Wire: count(u16 @ 0), target_sys(u8 @ 2), target_comp(u8 @ 3)
                            if pay_len < 3 { continue; }
                            let count = u16_le(&payload, 0) as usize;
                            dl_count.set(count as u16);
                            dl_next.set(0);
                            {
                                let mut pm = navigation::PENDING_MISSION.lock().await;
                                pm.count = 0;
                                pm.ready = false;
                            }
                            if count == 0 {
                                pending_mission_ack.set(Some(MAV_MISSION_ACCEPTED));
                            } else if count <= MAX_WAYPOINTS {
                                pending_mission_req.set(Some(0));
                                info!("Mission upload: {} waypoints incoming", count);
                            } else {
                                warn!("Mission too large: {} > {}", count, MAX_WAYPOINTS);
                                pending_mission_ack.set(Some(1)); // MAV_MISSION_NO_SPACE
                            }
                        }

                        73 => {
                            // MISSION_ITEM_INT — store waypoint, request next or finalize
                            // Wire: 4×f32(0-15), x=i32(16), y=i32(20), z=f32(24),
                            //       seq=u16(28), cmd=u16(30), target_sys(32),
                            //       target_comp(33), frame(34), current(35), autocont(36)
                            if pay_len < 37 { continue; }
                            let wp_seq = u16_le(&payload, 28) as usize;
                            let command = u16_le(&payload, 30);
                            let hold_s = f32_le(&payload, 0).max(0.0); // param1 = hold time

                            // Only NAV_WAYPOINT (16) creates a flight waypoint
                            if command == 16 && wp_seq < MAX_WAYPOINTS {
                                let lat_deg = (i32_le(&payload, 16) as f64) / 1e7;
                                let lon_deg = (i32_le(&payload, 20) as f64) / 1e7;
                                let alt_m   = f32_le(&payload, 24);
                                let mut pm = navigation::PENDING_MISSION.lock().await;
                                pm.waypoints[wp_seq] = Waypoint {
                                    position: LatLonAlt { lat_deg, lon_deg, alt_m },
                                    hold_time_s: hold_s,
                                };
                            }

                            let next = (wp_seq + 1) as u16;
                            dl_next.set(next);
                            if next < dl_count.get() {
                                pending_mission_req.set(Some(next));
                            } else {
                                // All items received — mark ready
                                {
                                    let mut pm = navigation::PENDING_MISSION.lock().await;
                                    pm.count = dl_count.get() as usize;
                                    pm.ready = true;
                                }
                                pending_mission_ack.set(Some(MAV_MISSION_ACCEPTED));
                                info!("Mission upload complete ({} WPs)", dl_count.get());
                            }
                        }

                        76 => {
                            // COMMAND_LONG
                            if pay_len < 30 { continue; }
                            let param1 = f32_le(&payload, 0);
                            let param2 = f32_le(&payload, 4);
                            let cmd_id = u16_le(&payload, 28);
                            let result = handle_command(cmd_id, param1, param2).await;
                            pending_ack.set(Some((cmd_id, result)));
                        }

                        _ => {}
                    }
                }
                _ => { sm = 0; }
            }
        }
    };

    join(tx_fut, rx_fut).await;
}
