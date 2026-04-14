//! RC receiver input — SBUS parser on USART2.
//!
//! SBUS: 100 kbaud, 8E2, inverted logic (requires hardware inverter or
//! STM32 USART inversion feature).
//! Frame: 25 bytes — start 0x0F, 22 bytes channel data (16 × 11-bit), flags, end 0x00.

use embassy_stm32::{
    peripherals,
    usart::{Config as UartConfig, Uart},
};
use embassy_time::{Duration, Instant};
use defmt::{info, warn};

use crate::{
    types::{FlightMode, RcInput},
    Irqs, STATE,
};

const SBUS_FRAME_LEN:    usize = 25;
const SBUS_START_BYTE:   u8    = 0x0F;
const SBUS_END_BYTE:     u8    = 0x00;
const SBUS_FLAG_FAILSAFE: u8   = 1 << 3;
const SBUS_FLAG_FRAME_LOST: u8 = 1 << 2;

const SBUS_MIN: u16 = 172;
const SBUS_MAX: u16 = 1811;

/// 16 SBUS channel values (raw 11-bit, 172–1811).
#[derive(Default)]
struct SbusFrame {
    channels: [u16; 16],
    failsafe: bool,
    frame_lost: bool,
}

fn parse_sbus(buf: &[u8; SBUS_FRAME_LEN]) -> Option<SbusFrame> {
    if buf[0] != SBUS_START_BYTE || buf[24] != SBUS_END_BYTE {
        return None;
    }

    let b = &buf[1..23]; // 22 data bytes
    let mut ch = [0u16; 16];

    // Unpack 16 × 11-bit channels from 22 bytes (little-endian bit packing)
    ch[0]  = ((b[0] as u16)       | ((b[1] as u16) << 8))                    & 0x07FF;
    ch[1]  = (((b[1] as u16) >> 3) | ((b[2] as u16) << 5))                   & 0x07FF;
    ch[2]  = (((b[2] as u16) >> 6) | ((b[3] as u16) << 2) | ((b[4] as u16) << 10)) & 0x07FF;
    ch[3]  = (((b[4] as u16) >> 1) | ((b[5] as u16) << 7))                   & 0x07FF;
    ch[4]  = (((b[5] as u16) >> 4) | ((b[6] as u16) << 4))                   & 0x07FF;
    ch[5]  = (((b[6] as u16) >> 7) | ((b[7] as u16) << 1) | ((b[8] as u16) << 9)) & 0x07FF;
    ch[6]  = (((b[8] as u16) >> 2) | ((b[9] as u16) << 6))                   & 0x07FF;
    ch[7]  = (((b[9] as u16) >> 5) | ((b[10] as u16) << 3))                  & 0x07FF;
    ch[8]  = ((b[11] as u16)       | ((b[12] as u16) << 8))                   & 0x07FF;
    ch[9]  = (((b[12] as u16) >> 3) | ((b[13] as u16) << 5))                  & 0x07FF;
    ch[10] = (((b[13] as u16) >> 6) | ((b[14] as u16) << 2) | ((b[15] as u16) << 10)) & 0x07FF;
    ch[11] = (((b[15] as u16) >> 1) | ((b[16] as u16) << 7))                  & 0x07FF;
    ch[12] = (((b[16] as u16) >> 4) | ((b[17] as u16) << 4))                  & 0x07FF;
    ch[13] = (((b[17] as u16) >> 7) | ((b[18] as u16) << 1) | ((b[19] as u16) << 9)) & 0x07FF;
    ch[14] = (((b[19] as u16) >> 2) | ((b[20] as u16) << 6))                  & 0x07FF;
    ch[15] = (((b[20] as u16) >> 5) | ((b[21] as u16) << 3))                  & 0x07FF;

    let flags = buf[23];
    Some(SbusFrame {
        channels: ch,
        failsafe:   flags & SBUS_FLAG_FAILSAFE   != 0,
        frame_lost: flags & SBUS_FLAG_FRAME_LOST != 0,
    })
}

/// Normalise an 11-bit SBUS value to [-1.0, 1.0].
fn normalise(raw: u16) -> f32 {
    let clamped = raw.clamp(SBUS_MIN, SBUS_MAX);
    (clamped as f32 - SBUS_MIN as f32) / (SBUS_MAX - SBUS_MIN) as f32 * 2.0 - 1.0
}

/// Normalise to [0.0, 1.0] (throttle channel).
fn normalise_01(raw: u16) -> f32 {
    (normalise(raw) + 1.0) * 0.5
}

/// Decode flight mode from a 3-position switch channel (ch5).
fn decode_mode(raw: u16) -> FlightMode {
    match normalise(raw) {
        v if v < -0.5 => FlightMode::Stabilise,
        v if v <  0.5 => FlightMode::AltitudeHold,
        _              => FlightMode::Auto,
    }
}

// ---------------------------------------------------------------------------
// Embassy task
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn rc_task(
    uart_peri: peripherals::USART2,
    tx_pin:    peripherals::PA2,
    rx_pin:    peripherals::PA3,
    rx_dma:    peripherals::DMA1_CH5,
    irqs:      Irqs,
) {
    let mut cfg = UartConfig::default();
    cfg.baudrate         = 100_000;
    cfg.parity           = embassy_stm32::usart::Parity::ParityEven;
    cfg.stop_bits        = embassy_stm32::usart::StopBits::STOP2;
    // NOTE: SBUS requires inverted RX logic. embassy-stm32 does not expose invert_rx
    // for STM32F4 — use a hardware signal inverter (e.g. 74HC04) on the RX line.
    cfg.data_bits        = embassy_stm32::usart::DataBits::DataBits8; // 8E2 → 9-bit USART

    let mut uart = Uart::new(uart_peri, rx_pin, tx_pin, irqs,
                             embassy_stm32::dma::NoDma, rx_dma, cfg).unwrap();

    info!("RC task started (SBUS @ 100 kbaud)");

    let mut buf = [0u8; SBUS_FRAME_LEN];
    let mut last_frame = Instant::now();

    loop {
        // Read until start byte
        let mut sync = [0u8; 1];
        uart.read(&mut sync).await.ok();
        if sync[0] != SBUS_START_BYTE { continue; }

        buf[0] = SBUS_START_BYTE;
        if uart.read(&mut buf[1..]).await.is_err() { continue; }

        let now = Instant::now();
        let failsafe = now.duration_since(last_frame) > Duration::from_millis(500);

        if let Some(frame) = parse_sbus(&buf) {
            last_frame = now;

            let input = RcInput {
                throttle: normalise_01(frame.channels[2]),
                roll:     normalise(frame.channels[0]),
                pitch:    -normalise(frame.channels[1]), // pitch inverted on most TX
                yaw:      normalise(frame.channels[3]),
                arm:      normalise(frame.channels[4]) > 0.5,
                mode:     decode_mode(frame.channels[5]),
                failsafe: frame.failsafe || frame.frame_lost || failsafe,
            };

            *STATE.rc_input.lock().await = input;

            // Safety: disarm if failsafe triggered
            if input.failsafe {
                warn!("RC failsafe — disarming");
                *STATE.armed.lock().await = false;
            }
        }
    }
}
