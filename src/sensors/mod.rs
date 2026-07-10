pub mod baro;
pub mod battery;
pub mod flow;
pub mod gps;
pub mod imu;
pub mod mag;
pub mod rc;

use embassy_stm32::usart::{Error, RingBufferedUartRx};

/// Fill `buf` completely from a ring-buffered DMA RX (one-shot `UartRx::read`
/// drops bytes between reads and corrupts frames). Err on UART error -> caller resyncs.
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
