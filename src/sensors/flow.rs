//! MTF-02P optical flow + rangefinder — UART4 RX-only, 19200 baud.
//!
//! Frame format (13 bytes total):
//!   0xFE 0x0A                        — sync header
//!   flow_x_h  flow_x_l              — int16 big-endian, sensor flow units
//!   flow_y_h  flow_y_l              — int16 big-endian
//!   quality_h quality_l             — uint16 big-endian (0–255 usable range)
//!   range_3 range_2 range_1 range_0 — int32 big-endian, mm; −1 = no data
//!   checksum                         — XOR of the 10 data bytes
//!
//! Flow scale: sensor outputs in units of 0.01 mrad/s; multiply × 100
//! to convert to FlowData's mrad/s × 1000 representation.
//!
//! Pinout: PC11 = UART4 RX (AF8). No TX needed.

use defmt::{info, warn};
use embassy_stm32::{
    peripherals,
    usart::{Config, UartRx},
    Peri,
};
use embassy_time::{Duration, with_timeout};

use crate::{types::FlowData, Irqs, STATE};

const SYNC1: u8 = 0xFE;
const SYNC2: u8 = 0x0A;
const DATA_LEN: usize = 10;
const QUALITY_THRESHOLD: u16 = 50;

// Sensor flow unit → mrad/s × 1000 (FlowData representation)
const FLOW_SCALE: i32 = 100;

#[embassy_executor::task]
pub async fn flow_task(
    uart: Peri<'static, peripherals::UART4>,
    rx:   Peri<'static, peripherals::PC11>,
    dma:  Peri<'static, peripherals::DMA1_CH2>,
    irqs: Irqs,
) {
    let mut cfg = Config::default();
    cfg.baudrate = 19200;

    let mut rx = UartRx::new(uart, rx, dma, irqs, cfg)
        .expect("UART4 (optical flow) init failed");

    info!("Flow: MTF-02P on UART4 @ 19200 baud");

    let mut byte = [0u8; 1];
    let mut data = [0u8; DATA_LEN];

    loop {
        // ── sync byte 1 ─────────────────────────────────────────────────────
        match with_timeout(Duration::from_millis(500), rx.read(&mut byte)).await {
            Ok(Ok(())) if byte[0] == SYNC1 => {}
            _ => continue,
        }

        // ── sync byte 2 ─────────────────────────────────────────────────────
        match with_timeout(Duration::from_millis(50), rx.read(&mut byte)).await {
            Ok(Ok(())) if byte[0] == SYNC2 => {}
            _ => continue,
        }

        // ── 10 data bytes ───────────────────────────────────────────────────
        if with_timeout(Duration::from_millis(50), rx.read(&mut data)).await != Ok(Ok(())) {
            continue;
        }

        // ── checksum ────────────────────────────────────────────────────────
        if with_timeout(Duration::from_millis(10), rx.read(&mut byte)).await != Ok(Ok(())) {
            continue;
        }
        let expected: u8 = data.iter().fold(0u8, |acc, &b| acc ^ b);
        if expected != byte[0] {
            warn!("Flow: checksum mismatch — dropped frame");
            continue;
        }

        let flow_x  = i16::from_be_bytes([data[0], data[1]]) as i32;
        let flow_y  = i16::from_be_bytes([data[2], data[3]]) as i32;
        let quality = u16::from_be_bytes([data[4], data[5]]);
        let height  = i32::from_be_bytes([data[6], data[7], data[8], data[9]]);

        let valid = quality >= QUALITY_THRESHOLD && height > 0;

        *STATE.flow.lock().await = FlowData {
            quality:      quality as u8,
            vel_x_mrad_s: flow_x * FLOW_SCALE,
            vel_y_mrad_s: flow_y * FLOW_SCALE,
            height_mm:    height,
            valid,
        };
    }
}
