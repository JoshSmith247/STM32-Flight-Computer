//! Servo output - 4-channel PWM on TIM4 at 50 Hz (S1-S4 = PD12-PD15, AF2).
//! Reads STATE.servo_outputs (0.0 = 1000 us retracted, 1.0 = 2000 us deployed).

use defmt::info;
use embassy_stm32::{
    gpio::OutputType,
    peripherals,
    time::hz,
    timer::{
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm},
    },
    Peri,
};
use embassy_time::{Duration, Ticker};

use crate::{types::payload_flags, STATE};

const SERVO_HZ: u32 = 50;
const PERIOD_US: u32 = 1_000_000 / SERVO_HZ; // 20_000 us
const MIN_US: u32 = 1000;
const MAX_US: u32 = 2000;

fn norm_to_duty(norm: f32, max_duty: u32) -> u32 {
    let us = MIN_US + (norm.clamp(0.0, 1.0) * (MAX_US - MIN_US) as f32) as u32;
    (us * max_duty) / PERIOD_US
}

#[embassy_executor::task]
pub async fn servo_task(
    tim:  Peri<'static, peripherals::TIM4>,
    pd12: Peri<'static, peripherals::PD12>,
    pd13: Peri<'static, peripherals::PD13>,
    pd14: Peri<'static, peripherals::PD14>,
    pd15: Peri<'static, peripherals::PD15>,
) {
    let ch1 = PwmPin::new(pd12, OutputType::PushPull);
    let ch2 = PwmPin::new(pd13, OutputType::PushPull);
    let ch3 = PwmPin::new(pd14, OutputType::PushPull);
    let ch4 = PwmPin::new(pd15, OutputType::PushPull);

    let pwm = SimplePwm::new(
        tim,
        Some(ch1), Some(ch2), Some(ch3), Some(ch4),
        hz(SERVO_HZ),
        CountingMode::EdgeAlignedUp,
    );

    let mut ch = pwm.split();
    let max = ch.ch1.max_duty_cycle();

    ch.ch1.set_duty_cycle(norm_to_duty(0.0, max));
    ch.ch2.set_duty_cycle(norm_to_duty(0.0, max));
    ch.ch3.set_duty_cycle(norm_to_duty(0.0, max));
    ch.ch4.set_duty_cycle(norm_to_duty(0.0, max));
    ch.ch1.enable();
    ch.ch2.enable();
    ch.ch3.enable();
    ch.ch4.enable();

    let mut flags = *STATE.payload_flags.lock().await;
    flags |= payload_flags::SERVO_OUTPUTS;
    *STATE.payload_flags.lock().await = flags;

    info!("servo: TIM4 50 Hz ready, max_duty={}", max);

    let mut ticker = Ticker::every(Duration::from_hz(SERVO_HZ as u64));
    loop {
        let s = *STATE.servo_outputs.lock().await;
        ch.ch1.set_duty_cycle(norm_to_duty(s.s1, max));
        ch.ch2.set_duty_cycle(norm_to_duty(s.s2, max));
        ch.ch3.set_duty_cycle(norm_to_duty(s.s3, max));
        ch.ch4.set_duty_cycle(norm_to_duty(s.s4, max));
        ticker.next().await;
    }
}
