// TODO: ICM-42688-P over SPI1 at 24 MHz, 2 kHz sample rate.
//       Read accel + gyro, feed into AHRS filter, write attitude quaternion
//       to shared state.

#[embassy_executor::task]
pub async fn imu_task() {
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}
