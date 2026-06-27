pub mod baro;
pub mod battery;
pub mod flow;
pub mod gps;
pub mod imu;
pub mod mag;
pub mod rc;

use embassy_stm32::usart::{Error, RingBufferedUartRx};

/// Fill `buf` completely from a ring-buffered DMA RX. The DMA continuously fills the
/// ring in the background, so no incoming bytes are dropped while the executor is busy
/// elsewhere — unlike one-shot `UartRx::read`, which loses anything that arrives between
/// reads (the bug that corrupted MAVLink frames before telemetry was ring-buffered).
/// `RingBufferedUartRx::read` returns however many bytes are available, so we loop until
/// the caller's fixed-size buffer is full. Returns Err on a UART error so the caller can
/// resync.
pub(crate) async fn read_exact_ring(
    rx: &mut RingBufferedUartRx<'_>,
    buf: &mut [u8],
) -> Result<(), Error> {
    let mut n = 0;
    while n < buf.len() {
        n += rx.read(&mut buf[n..]).await?;
    }
    Ok(())
}
