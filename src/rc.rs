//! RC receiver input — SBUS parser on USART2.
//!
//! SBUS: 100 kbaud, 8E2, inverted logic (requires hardware inverter or
//! STM32 USART inversion feature).
//! Frame: 25 bytes — start 0x0F, 22 bytes channel data (16 × 11-bit), flags, end 0x00.

use embassy_stm32::{
    peripherals, Peri,
    usart::{Config, DataBits, Parity, StopBits, UartRx},
};
use embassy_time::{with_timeout, Duration};
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

/// 100 ms = ~7× the 14 ms SBUS frame period; if no frame arrives in this
/// window the link is considered lost and inputs are zeroed with failsafe set.
const SBUS_TIMEOUT: Duration = Duration::from_millis(100);

#[embassy_executor::task]
pub async fn rc_task(
    uart_peri: Peri<'static, peripherals::USART2>,
    _tx_pin:   Peri<'static, peripherals::PA2>,
    rx_pin:    Peri<'static, peripherals::PA3>,
    rx_dma:    Peri<'static, peripherals::DMA1_CH5>,
    irqs:      Irqs,
) {
    let mut config = Config::default();
    config.baudrate  = 100_000;
    config.data_bits = DataBits::DataBits8;
    config.stop_bits = StopBits::STOP2;
    config.parity    = Parity::ParityEven;
    config.invert_rx = true;

    let mut rx = UartRx::new(uart_peri, rx_pin, rx_dma, irqs, config)
        .expect("USART2 init failed");

    info!("RC task started (SBUS @ 100 kbaud 8E2 inverted)");

    let mut buf = [0u8; SBUS_FRAME_LEN];

    loop {
        // read_until_idle fires when the line goes idle after the 25-byte burst,
        // which is the natural gap between SBUS frames (~14 ms period).
        match with_timeout(SBUS_TIMEOUT, rx.read_until_idle(&mut buf)).await {
            Ok(Ok(SBUS_FRAME_LEN)) => {
                if let Some(frame) = parse_sbus(&buf) {
                    let rc = if frame.failsafe {
                        RcInput { failsafe: true, ..Default::default() }
                    } else {
                        RcInput {
                            throttle: normalise_01(frame.channels[2]),
                            roll:     normalise(frame.channels[0]),
                            pitch:    normalise(frame.channels[1]),
                            yaw:      normalise(frame.channels[3]),
                            // Ch5: arm switch (high = armed)
                            arm:      frame.channels[4] > 1000,
                            // Ch6: 3-position mode switch
                            mode:     decode_mode(frame.channels[5]),
                            failsafe: frame.frame_lost,
                        }
                    };
                    *STATE.rc_input.lock().await = rc;
                }
            }
            Ok(Ok(_)) => {
                // Partial frame — we caught the start mid-stream; wait for next idle.
                warn!("SBUS: partial frame, re-syncing");
            }
            Ok(Err(_)) => {
                warn!("SBUS: UART error");
            }
            Err(_) => {
                // No frame within timeout — link lost.
                warn!("SBUS: signal lost, failsafe active");
                *STATE.rc_input.lock().await = RcInput { failsafe: true, ..Default::default() };
            }
        }
    }
}
