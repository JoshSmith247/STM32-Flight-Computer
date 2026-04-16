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

// Demo sequence — cycles through every state so the LED patterns are visible
// without any hardware attached. Remove once real sensor tasks drive the state.
const DEMO_STATES: &[FlightState] = &[
    FlightState::Idle,
    FlightState::Arming,
    FlightState::Armed,
    FlightState::Flying,
    FlightState::Landing,
    FlightState::Fault,
];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let led = Output::new(p.PG7, Level::Low, Speed::Low);

    spawner.spawn(led::led_task(led).unwrap());
    spawner.spawn(imu::imu_task().unwrap());
    spawner.spawn(motors::motors_task().unwrap());
    spawner.spawn(mavlink::mavlink_task().unwrap());

    let mut i = 0;
    loop {
        let s = DEMO_STATES[i];
        state::set(s);
        defmt::info!("Flight state -> {}", s);
        Timer::after(Duration::from_secs(5)).await;
        i = (i + 1) % DEMO_STATES.len();
    }
}
