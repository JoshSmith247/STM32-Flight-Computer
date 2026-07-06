//! Status LED — blink patterns per FlightState.
//!
//! Custom FC: PG7, active-LOW. Nucleo (`--features nucleo`): LD2/PE1, active-HIGH.

use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

use crate::state::{self, FlightState};

#[inline]
fn led_on(led: &mut Output<'static>) {
    if cfg!(feature = "nucleo") { led.set_high() } else { led.set_low() }
}

#[inline]
fn led_off(led: &mut Output<'static>) {
    if cfg!(feature = "nucleo") { led.set_low() } else { led.set_high() }
}

#[embassy_executor::task]
pub async fn led_task(mut led: Output<'static>) {
    loop {
        match state::get() {
            FlightState::Idle => {
                // Slow blink — 1 Hz
                led_on(&mut led);
                Timer::after(Duration::from_millis(500)).await;
                led_off(&mut led);
                Timer::after(Duration::from_millis(500)).await;
            }
            FlightState::Arming => {
                // Fast blink — 5 Hz
                led_on(&mut led);
                Timer::after(Duration::from_millis(100)).await;
                led_off(&mut led);
                Timer::after(Duration::from_millis(100)).await;
            }
            FlightState::Armed => {
                // Double pulse, then dark gap
                for _ in 0..2 {
                    led_on(&mut led);
                    Timer::after(Duration::from_millis(100)).await;
                    led_off(&mut led);
                    Timer::after(Duration::from_millis(50)).await;
                }
                Timer::after(Duration::from_millis(200)).await;
            }
            FlightState::Flying => {
                // Solid on
                led_on(&mut led);
                Timer::after(Duration::from_millis(100)).await;
            }
            FlightState::Landing => {
                // Triple pulse every 2 s
                for _ in 0..3 {
                    led_on(&mut led);
                    Timer::after(Duration::from_millis(100)).await;
                    led_off(&mut led);
                    Timer::after(Duration::from_millis(100)).await;
                }
                Timer::after(Duration::from_millis(1400)).await;
            }
            FlightState::Fault => {
                // Rapid strobe
                led_on(&mut led);
                Timer::after(Duration::from_millis(50)).await;
                led_off(&mut led);
                Timer::after(Duration::from_millis(50)).await;
            }
        }
    }
}
