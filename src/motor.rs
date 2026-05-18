//! Motor output — TIM3 SimplePwm (standard RC PWM), 50 Hz bring-up.
//!
//! Current: standard 50 Hz PWM, 1000–2000 µs pulse width.
//! Works with all ESCs; ESCs arm on the first low-throttle pulse train at power-on.
//!
//! DSHOT600 upgrade path: encode_dshot() / build_dshot_buffer() are retained.
//! Replace this SimplePwm section with TIM3 DMA burst writes to CCR1–4;
//! bit-timing constants are already correct for 200 MHz timer clock.
//!
//! Pinout (TIM3, AF2):
//!   M1 → PB4  (TIM3 CH1)   M3 → PB0  (TIM3 CH3)
//!   M2 → PB5  (TIM3 CH2)   M4 → PB1  (TIM3 CH4)

use defmt::info;
use embassy_stm32::{
    gpio::OutputType,
    peripherals,
    time::Hertz,
    timer::{
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm},
    },
    Peri,
};
use embassy_time::{Duration, Ticker};

use crate::STATE;

// ---------------------------------------------------------------------------
// DSHOT600 constants — 200 MHz TIM3 clock (APB1=100 MHz, prescaler DIV2 → ×2)
// ---------------------------------------------------------------------------

const DSHOT_BIT_TICKS: u16 = 333; // 200 MHz / 600 kHz
const DSHOT_T1H:       u16 = 250; // 75%   = 1250 ns
const DSHOT_T0H:       u16 = 125; // 37.5% =  625 ns
const DSHOT_FRAME_LEN: usize = 16;

// ---------------------------------------------------------------------------
// DSHOT frame helpers (unused until DMA upgrade)
// ---------------------------------------------------------------------------

#[allow(dead_code)]
fn encode_dshot(throttle_11bit: u16, telemetry: bool) -> u16 {
    let payload = (throttle_11bit << 1) | (telemetry as u16);
    let crc = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0F;
    (payload << 4) | crc
}

#[allow(dead_code)]
fn build_dshot_buffer(frame: u16, buf: &mut [u16; 18]) {
    for i in 0..DSHOT_FRAME_LEN {
        let bit = (frame >> (15 - i)) & 1;
        buf[i] = if bit == 1 { DSHOT_T1H } else { DSHOT_T0H };
    }
    buf[16] = 0;
    buf[17] = 0;
}

#[allow(dead_code)]
fn throttle_to_dshot(t: f32) -> u16 {
    if t <= 0.0 { return 0; }
    (48.0 + t * (2047.0 - 48.0)) as u16
}

// ---------------------------------------------------------------------------
// Embassy task
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn motor_task(
    tim: Peri<'static, peripherals::TIM3>,
    m1:  Peri<'static, peripherals::PB4>,
    m2:  Peri<'static, peripherals::PB5>,
    m3:  Peri<'static, peripherals::PB0>,
    m4:  Peri<'static, peripherals::PB1>,
) {
    let ch1 = PwmPin::new(m1, OutputType::PushPull);
    let ch2 = PwmPin::new(m2, OutputType::PushPull);
    let ch3 = PwmPin::new(m3, OutputType::PushPull);
    let ch4 = PwmPin::new(m4, OutputType::PushPull);

    let pwm = SimplePwm::new(
        tim,
        Some(ch1), Some(ch2), Some(ch3), Some(ch4),
        Hertz(50),
        CountingMode::EdgeAlignedUp,
    );

    // split() consumes pwm and returns four independent SimplePwmChannel structs
    // with 'static lifetime — valid here because all Peri args are 'static.
    let mut ch = pwm.split();

    // 1000 µs = minimum throttle (5% of 20 ms period).
    // Hold here so ESCs can complete their startup/arming beep sequence.
    ch.ch1.set_duty_cycle_fraction(1000, 20_000);
    ch.ch2.set_duty_cycle_fraction(1000, 20_000);
    ch.ch3.set_duty_cycle_fraction(1000, 20_000);
    ch.ch4.set_duty_cycle_fraction(1000, 20_000);

    ch.ch1.enable();
    ch.ch2.enable();
    ch.ch3.enable();
    ch.ch4.enable();

    info!("Motors: 50 Hz PWM started (1000–2000 µs)");

    let mut ticker = Ticker::every(Duration::from_hz(500));

    loop {
        ticker.next().await;

        let outputs  = *STATE.motor_outputs.lock().await;
        let is_armed = *STATE.armed.lock().await;

        // Map normalised throttle [0, 1] → pulse width [1000, 2000] µs.
        // set_duty_cycle_fraction(num, denom) sets duty to num/denom of the period.
        // Period = 20 000 µs at 50 Hz, so num = pulse_us gives exact µs control.
        let pulse = |t: f32| 1000 + (t.clamp(0.0, 1.0) * 1000.0) as u32;

        let (p1, p2, p3, p4) = if is_armed {
            (pulse(outputs.m1), pulse(outputs.m2), pulse(outputs.m3), pulse(outputs.m4))
        } else {
            (1000, 1000, 1000, 1000)
        };

        ch.ch1.set_duty_cycle_fraction(p1, 20_000);
        ch.ch2.set_duty_cycle_fraction(p2, 20_000);
        ch.ch3.set_duty_cycle_fraction(p3, 20_000);
        ch.ch4.set_duty_cycle_fraction(p4, 20_000);
    }
}
