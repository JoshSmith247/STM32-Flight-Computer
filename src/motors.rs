// TODO: DSHOT600 output for 4 motors via timer DMA.
//       Receive throttle commands from flight controller, encode as DSHOT frames,
//       and write to ESCs.

#[embassy_executor::task]
pub async fn motors_task(at: AttitudeSetpoint) {

    //motor::motor_task();

    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}
