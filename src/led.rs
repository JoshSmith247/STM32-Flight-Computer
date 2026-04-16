use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

use crate::state::{self, FlightState};

#[embassy_executor::task]
pub async fn led_task(mut led: Output<'static>) {
    loop {
        match state::get() {
            FlightState::Idle => {
                // Slow blink — 1 Hz
                led.set_high();
                Timer::after(Duration::from_millis(500)).await;
                led.set_low();
                Timer::after(Duration::from_millis(500)).await;
            }
            FlightState::Arming => {
                // Fast blink — 5 Hz
                led.set_high();
                Timer::after(Duration::from_millis(100)).await;
                led.set_low();
                Timer::after(Duration::from_millis(100)).await;
            }
            FlightState::Armed => {
                // Double pulse every 2 s
                for _ in 0..2 {
                    led.set_high();
                    Timer::after(Duration::from_millis(100)).await;
                    led.set_low();
                    Timer::after(Duration::from_millis(100)).await;
                }
                Timer::after(Duration::from_millis(1600)).await;
            }
            FlightState::Flying => {
                // Solid on
                led.set_high();
                Timer::after(Duration::from_millis(100)).await;
            }
            FlightState::Landing => {
                // Triple pulse every 2 s
                for _ in 0..3 {
                    led.set_high();
                    Timer::after(Duration::from_millis(100)).await;
                    led.set_low();
                    Timer::after(Duration::from_millis(100)).await;
                }
                Timer::after(Duration::from_millis(1400)).await;
            }
            FlightState::Fault => {
                // Rapid strobe
                led.set_high();
                Timer::after(Duration::from_millis(50)).await;
                led.set_low();
                Timer::after(Duration::from_millis(50)).await;
            }
        }
    }
}
