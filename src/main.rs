#![no_std]
#![no_main]

mod ahrs;
mod baro;
mod imu;
mod led;
mod motor;
mod motors;
mod navigation;
mod pid;
mod rc;
mod state;
mod telemetry;
mod types;

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
};
use embassy_time::{Duration, Ticker, Timer};
use state::FlightState;
use {defmt_rtt as _, panic_probe as _};

pub static STATE: types::SharedState = types::SharedState::new();

bind_interrupts!(pub struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
    USART3 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART3>;
});

/// 500 Hz control loop: AHRS fusion → PID cascade → motor mix → STATE.motor_outputs.
#[embassy_executor::task]
async fn control_task() {
    use ahrs::MadgwickFilter;
    use pid::{mix_quad_x, FlightPids};

    let mut filter = MadgwickFilter::new(0.1, 1.0 / 500.0);
    let mut pids = FlightPids::default();
    let mut ticker = Ticker::every(Duration::from_hz(500));

    loop {
        ticker.next().await;

        let imu      = *STATE.imu_data.lock().await;
        let is_armed = *STATE.armed.lock().await;
        let nav_cmd  = *STATE.nav_command.lock().await;
        let rc       = *STATE.rc_input.lock().await;

        let setpoint = if nav_cmd.autonomous {
            nav_cmd.attitude_setpoint
        } else {
            rc.to_attitude_setpoint()
        };

        let quat = filter.update(imu.gyro, imu.accel);
        *STATE.attitude.lock().await = quat;
        let euler = filter.euler();

        if !is_armed {
            pids.reset_all();
            *STATE.motor_outputs.lock().await = Default::default();
            continue;
        }

        let (roll_out, pitch_out, yaw_out) = pids.update(&setpoint, &euler, imu.gyro);
        let outputs = mix_quad_x(setpoint.throttle, roll_out, pitch_out, yaw_out);
        *STATE.motor_outputs.lock().await = outputs;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let led = Output::new(p.PG7, Level::Low, Speed::Low);

    spawner.spawn(led::led_task(led).unwrap());
    spawner.spawn(imu::imu_task().unwrap());
    spawner.spawn(motor::motor_task(p.TIM3, p.PB4, p.PB5, p.PB0, p.PB1).unwrap());
    spawner.spawn(rc::rc_task(p.USART2, p.PA2, p.PA3, p.DMA1_CH5, Irqs).unwrap());
    spawner.spawn(telemetry::telemetry_task(p.USART3, p.PB10, p.PB11, p.DMA1_CH3, Irqs).unwrap());
    spawner.spawn(baro::baro_task(p.PA8).unwrap());
    spawner.spawn(navigation::navigation_task().unwrap());
    spawner.spawn(control_task().unwrap());

    state::set(FlightState::Idle);

    Timer::after(Duration::from_millis(10_000)).await; // pre-arm countdown

    state::set(FlightState::Arming);
    *STATE.armed.lock().await = true;
    state::set(FlightState::Armed);

    Timer::after(Duration::from_millis(10_000)).await; // post-arm hold

    state::set(FlightState::Flying);

    loop {
        Timer::after(Duration::from_millis(1_000)).await;
    }
}
