// TODO: MAVLink v2 over UART.
//       Send HEARTBEAT, ATTITUDE, and LOCAL_POSITION_NED at appropriate rates.
//       Receive COMMAND_LONG messages (arm/disarm, mode change, etc.) and
//       forward them to the flight state machine.

#[embassy_executor::task]
pub async fn mavlink_task() {
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}
