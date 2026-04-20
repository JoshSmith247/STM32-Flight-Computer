// Placeholder — DSHOT600 output is implemented in motor.rs.
// This task is currently unused; motor::motor_task reads STATE.motor_outputs directly.

#[embassy_executor::task]
pub async fn motors_task() {
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}
