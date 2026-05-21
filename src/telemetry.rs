//! MAVLink v2 telemetry — USART3 TX at 57600 baud.
//!
//! Wire:  STM32 PB10 (TX) → Pi GPIO15 (RXD)
//!        STM32 PB11 (RX) → Pi GPIO14 (TXD)   [RX deferred — no RX DMA allocated]
//!
//! Messages sent:
//!   HEARTBEAT (#0)   1 Hz  — armed state, flight mode
//!   ATTITUDE  (#30) 10 Hz  — roll/pitch/yaw from AHRS + body rates

use defmt::info;
use embassy_stm32::{
    peripherals,
    usart::{Config, UartTx},
    Peri,
};
use embassy_time::{Duration, Ticker};

use crate::{
    types::{FlightMode, Quaternion},
    STATE,
};

const MAVLINK_SYSTEM_ID:     u8 = 1;
const MAVLINK_COMPONENT_ID:  u8 = 1;
const MAV_TYPE_QUAD:         u8 = 2;
const MAV_AUTOPILOT_GENERIC: u8 = 0;

// ---------------------------------------------------------------------------
// Frame builder — writes into caller-supplied buffer, returns frame length.
// MAVLink v2: 10-byte header + N-byte payload + 2-byte CRC = 12 + N bytes.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Payload builders
// ---------------------------------------------------------------------------

fn heartbeat_payload(armed: bool, mode: FlightMode) -> [u8; 9] {
    let custom = mode as u32;
    [
        (custom & 0xFF) as u8,
        ((custom >>  8) & 0xFF) as u8,
        ((custom >> 16) & 0xFF) as u8,
        ((custom >> 24) & 0xFF) as u8,
        MAV_TYPE_QUAD,
        MAV_AUTOPILOT_GENERIC,
        if armed { 0x80 } else { 0x00 }, // MAV_MODE_FLAG_SAFETY_ARMED
        if armed { 4 } else { 3 },        // MAV_STATE_ACTIVE / STANDBY
        2,                                // MAVLink version
    ]
}

// ATTITUDE (#30) payload: time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
fn attitude_payload(
    time_ms: u32,
    roll: f32, pitch: f32, yaw: f32,
    rollspeed: f32, pitchspeed: f32, yawspeed: f32,
) -> [u8; 28] {
    let mut p = [0u8; 28];
    p[0..4].copy_from_slice(&time_ms.to_le_bytes());
    p[4..8].copy_from_slice(&roll.to_bits().to_le_bytes());
    p[8..12].copy_from_slice(&pitch.to_bits().to_le_bytes());
    p[12..16].copy_from_slice(&yaw.to_bits().to_le_bytes());
    p[16..20].copy_from_slice(&rollspeed.to_bits().to_le_bytes());
    p[20..24].copy_from_slice(&pitchspeed.to_bits().to_le_bytes());
    p[24..28].copy_from_slice(&yawspeed.to_bits().to_le_bytes());
    p
}

fn quat_to_euler(q: Quaternion) -> (f32, f32, f32) {
    let roll  = libm::atan2f(2.0*(q.w*q.x + q.y*q.z), 1.0 - 2.0*(q.x*q.x + q.y*q.y));
    let pitch = libm::asinf((2.0*(q.w*q.y - q.z*q.x)).clamp(-1.0, 1.0));
    let yaw   = libm::atan2f(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
    (roll, pitch, yaw)
}

// ---------------------------------------------------------------------------
// Embassy task
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn telemetry_task(
    uart_peri: Peri<'static, peripherals::USART3>,
    tx_pin:    Peri<'static, peripherals::PB10>,
    _rx_pin:   Peri<'static, peripherals::PB11>,
    tx_dma:    Peri<'static, peripherals::DMA1_CH3>,
    _irqs:     crate::Irqs,
) {
    let mut config = Config::default();
    config.baudrate = 57600;

    let mut tx = UartTx::new(uart_peri, tx_pin, tx_dma, config)
        .expect("USART3 TX init failed");

    info!("Telemetry task started (MAVLink v2 @ 57600)");

    let mut seq:     u8   = 0;
    let mut tick:    u8   = 0;
    let mut time_ms: u32  = 0;
    let mut buf = [0u8; 64]; // 64 > 40 bytes (max ATTITUDE frame)

    // Base rate 10 Hz; HEARTBEAT sent every 10th tick (1 Hz).
    let mut ticker = Ticker::every(Duration::from_hz(10));

    loop {
        ticker.next().await;
        time_ms = time_ms.wrapping_add(100);

        let quat  = *STATE.attitude.lock().await;
        let imu   = *STATE.imu_data.lock().await;
        let armed = *STATE.armed.lock().await;
        let mode  = STATE.rc_input.lock().await.mode;

        // 10 Hz: ATTITUDE (msg_id=30, CRC_EXTRA=39)
        let (roll, pitch, yaw) = quat_to_euler(quat);
        let payload = attitude_payload(
            time_ms, roll, pitch, yaw,
            imu.gyro.x, imu.gyro.y, imu.gyro.z,
        );
        let len = write_frame(&mut buf, seq, 30, &payload, 39);
        tx.write(&buf[..len]).await.ok();
        seq = seq.wrapping_add(1);

        // 1 Hz: HEARTBEAT (msg_id=0, CRC_EXTRA=50)
        tick = tick.wrapping_add(1);
        if tick >= 10 {
            tick = 0;
            let payload = heartbeat_payload(armed, mode);
            let len = write_frame(&mut buf, seq, 0, &payload, 50);
            tx.write(&buf[..len]).await.ok();
            seq = seq.wrapping_add(1);
        }
    }
}
