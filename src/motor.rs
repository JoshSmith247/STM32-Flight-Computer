//! Motor output driver — DSHOT600 via TIM3 DMA on STM32F405.
//!
//! DSHOT600 protocol: 600 kbit/s digital ESC protocol.
//!   - Each frame: 16 bits (11-bit throttle 0–2047, 1-bit telemetry request, 4-bit CRC)
//!   - Bit timing: ~1.67 µs/bit; '1' = 75% duty, '0' = 37.5% duty
//!   - No calibration required (unlike PWM)
//!
//! Pinout (TIM3, APB1 @ 84 MHz):
//!   M1 → PB4  (TIM3 CH1)
//!   M2 → PB5  (TIM3 CH2)
//!   M3 → PB0  (TIM3 CH3)
//!   M4 → PB1  (TIM3 CH4)

use embassy_stm32::peripherals;
use embassy_time::{Duration, Ticker};
use defmt::info;

use crate::{types::MotorOutputs, STATE};

// DSHOT600 @ 84 MHz APB1 timer clock
// Period = 84 MHz / 600 kHz / 1 = 140 timer ticks per bit
const DSHOT_BIT_TICKS: u16 = 140;
const DSHOT_T1H:       u16 = 105; // 75% duty
const DSHOT_T0H:       u16 = 52;  // 37% duty
const DSHOT_FRAME_LEN: usize = 16;

/// Encode a DSHOT throttle value (0–2047) into a 16-bit frame with CRC.
fn encode_dshot(throttle_11bit: u16, telemetry: bool) -> u16 {
    let payload = (throttle_11bit << 1) | (telemetry as u16);
    let crc = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0F;
    (payload << 4) | crc
}

/// Build a DMA buffer of compare values for one motor channel.
/// `buf` must be at least DSHOT_FRAME_LEN + 2 elements (trailing zeros = low).
fn build_dshot_buffer(frame: u16, buf: &mut [u16; 18]) {
    for i in 0..DSHOT_FRAME_LEN {
        let bit = (frame >> (15 - i)) & 1;
        buf[i] = if bit == 1 { DSHOT_T1H } else { DSHOT_T0H };
    }
    buf[16] = 0;
    buf[17] = 0;
}

/// Normalised throttle (0.0–1.0) → DSHOT value (48–2047).
/// Values 0–47 are reserved for special commands (disarm, direction, etc.).
fn throttle_to_dshot(t: f32) -> u16 {
    if t <= 0.0 { return 0; } // DSHOT disarm command
    (48.0 + t * (2047.0 - 48.0)) as u16
}

// ---------------------------------------------------------------------------
// Embassy task
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn motor_task(
    tim:  peripherals::TIM3,
    m1:   peripherals::PB4,
    m2:   peripherals::PB5,
    m3:   peripherals::PB0,
    m4:   peripherals::PB1,
) {
    // TODO: Initialise TIM3 in PWM mode with DMA bursts for DSHOT.
    //       Embassy-stm32 does not yet have a first-class DSHOT driver;
    //       use SimplePwm for initial bring-up with standard PWM (50 Hz, 1000–2000 µs).

    info!("Motor task started (DSHOT600 stub)");

    let armed = false;
    let mut ticker = Ticker::every(Duration::from_hz(500));

    loop {
        ticker.next().await;

        let outputs = *STATE.motor_outputs.lock().await;
        let is_armed = *STATE.armed.lock().await;

        let (v1, v2, v3, v4) = if is_armed {
            (outputs.m1, outputs.m2, outputs.m3, outputs.m4)
        } else {
            (0.0, 0.0, 0.0, 0.0)
        };

        // TODO: write DSHOT frames via DMA to TIM3 CH1-4
        let _ = (throttle_to_dshot(v1), throttle_to_dshot(v2),
                 throttle_to_dshot(v3), throttle_to_dshot(v4));
    }
}
