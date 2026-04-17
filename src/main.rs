#![no_std]
#![no_main]

mod imu;
mod led;
mod mavlink;
mod motors;
mod state;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};
use state::FlightState;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let led = Output::new(p.PG7, Level::Low, Speed::Low);

    let tick_rate = 5;

    spawner.spawn(led::led_task(led).unwrap());
    spawner.spawn(imu::imu_task().unwrap());
    spawner.spawn(motors::motors_task().unwrap());
    spawner.spawn(mavlink::mavlink_task().unwrap());

    state::set(FlightState::Idle); // Default State

    Timer::after(Duration::from_millis(10000)).await; // 10-second countdown before arming

    state::set(FlightState::Arming); // Auto-arm (triggered when pin is pulled)

    Timer::after(Duration::from_millis(10000)).await; // 10-second countdown once armed

    // Fly!!!
    // Ascend to starting altitude

    

    loop {
        // Main action loop

        Timer::after(Duration::from_millis(tick_rate)).await;
    }
}
