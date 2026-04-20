//! MAVLink telemetry — USART3 at 57600 baud.
//!
//! Sends a MAVLink v2 stream to a GCS (Mission Planner / QGroundControl).
//!
//! Messages streamed at 100 Hz:
//!   HEARTBEAT          (1 Hz)
//!   ATTITUDE           (50 Hz) — roll/pitch/yaw from AHRS
//!   SCALED_IMU         (50 Hz) — raw accel + gyro
//!   VFR_HUD            (10 Hz) — altitude, airspeed, heading, throttle
//!   GPS_RAW_INT        (5 Hz)  — GPS fix + coordinates
//!   SYS_STATUS         (1 Hz)  — battery, sensor health flags
//!
//! Receives:
//!   COMMAND_LONG       — ARM/DISARM, mode changes, mission upload trigger
//!   MISSION_ITEM_INT   — waypoint upload
//!   SET_MODE           — flight mode override

use defmt::info;
use embassy_stm32::{peripherals, Peri};
use embassy_time::Duration;

use crate::{types::FlightMode, Irqs, STATE};

const MAVLINK_SYSTEM_ID:    u8 = 1;
const MAVLINK_COMPONENT_ID: u8 = 1;   // MAV_COMP_ID_AUTOPILOT1
const MAVLINK_VERSION:      u8 = 2;

/// MAV_TYPE_QUADROTOR = 2, MAV_AUTOPILOT_GENERIC = 0
const MAV_TYPE_QUAD: u8        = 2;
const MAV_AUTOPILOT_GENERIC: u8 = 0;

/// MAV_STATE_*
#[repr(u8)]
enum MavState {
    Standby = 3,
    Active  = 4,
}

// Minimal MAVLink v2 frame builder (without the `mavlink` crate's full codegen).
// For production use, prefer the `mavlink` crate with the `common` feature.
fn build_heartbeat(seq: u8, armed: bool, mode: FlightMode) -> [u8; 17] {
    // HEARTBEAT (#0): type, autopilot, base_mode, custom_mode, system_status, mavlink_version
    let base_mode: u8 = if armed { 0x80 } else { 0x00 }; // MAV_MODE_FLAG_SAFETY_ARMED
    let sys_status: u8 = if armed { MavState::Active as u8 } else { MavState::Standby as u8 };
    let custom_mode: u32 = mode as u32;

    let payload: [u8; 9] = [
        (custom_mode & 0xFF) as u8,
        ((custom_mode >> 8) & 0xFF) as u8,
        ((custom_mode >> 16) & 0xFF) as u8,
        ((custom_mode >> 24) & 0xFF) as u8,
        MAV_TYPE_QUAD,
        MAV_AUTOPILOT_GENERIC,
        base_mode,
        sys_status,
        MAVLINK_VERSION,
    ];

    build_mavlink_frame(seq, 0, &payload) // msg id 0 = HEARTBEAT
}

/// Assemble a MAVLink v2 frame: STX + len + incompat + compat + seq + sysid + compid + msgid(3) + payload + CRC(2)
fn build_mavlink_frame(seq: u8, msg_id: u32, payload: &[u8]) -> [u8; 17] {
    // Fixed-size output for heartbeat (9-byte payload → 17-byte frame)
    let mut frame = [0u8; 17];
    frame[0] = 0xFD; // MAVLink v2 STX
    frame[1] = payload.len() as u8;
    frame[2] = 0;    // incompat flags
    frame[3] = 0;    // compat flags
    frame[4] = seq;
    frame[5] = MAVLINK_SYSTEM_ID;
    frame[6] = MAVLINK_COMPONENT_ID;
    frame[7] = (msg_id & 0xFF) as u8;
    frame[8] = ((msg_id >> 8) & 0xFF) as u8;
    frame[9] = ((msg_id >> 16) & 0xFF) as u8;
    frame[10..10 + payload.len()].copy_from_slice(payload);

    // MAVLink CRC-16/MCRF4XX over header (excluding STX) + payload
    let crc = mavlink_crc(&frame[1..10 + payload.len()], 50); // 50 = HEARTBEAT CRC_EXTRA
    frame[10 + payload.len()]     = (crc & 0xFF) as u8;
    frame[10 + payload.len() + 1] = (crc >> 8) as u8;
    frame
}

fn mavlink_crc(data: &[u8], extra: u8) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        let mut tmp = byte ^ (crc as u8);
        tmp ^= tmp << 4;
        crc = (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4);
    }
    // Include CRC_EXTRA
    let mut tmp = extra ^ (crc as u8);
    tmp ^= tmp << 4;
    crc = (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp as u16) >> 4);
    crc
}

// ---------------------------------------------------------------------------
// Embassy task
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn telemetry_task(
    uart_peri: Peri<'static, peripherals::USART3>,
    tx_pin:    Peri<'static, peripherals::PB10>,
    rx_pin:    Peri<'static, peripherals::PB11>,
    tx_dma:    Peri<'static, peripherals::DMA1_CH3>,
    irqs:      Irqs,
) {
    // TODO: init USART3 @ 57600 with TX DMA for MAVLink v2.
    // Uart::new API changed in newer Embassy — revisit when wiring hardware.
    let _ = (uart_peri, rx_pin, tx_pin, tx_dma, irqs);
    info!("Telemetry task started (MAVLink v2 stub)");

    loop {
        embassy_time::Timer::after(Duration::from_secs(1)).await;
    }
}
