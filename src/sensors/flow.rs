//! MicoAir MTF-02P optical flow + rangefinder - UART4 RX-only (PC11), 115200 baud,
//! MICOLINK protocol, 50 Hz. Parses the msg 0x51 range/flow frame into FlowData.
//! WARNING: BRING-UP: verify flow axis polarity vs body forward/right after mounting.

use defmt::{info, warn};
use embassy_stm32::{
    peripherals,
    usart::{Config, UartRx},
    Peri,
};
use embassy_time::{Duration, with_timeout};

use crate::{types::FlowData, Irqs, STATE};

const MICOLINK_HEAD:      u8 = 0xEF;
const MSG_ID_RANGE_FLOW:  u8 = 0x51;
const RANGE_FLOW_LEN:     usize = 20;
const MAX_PAYLOAD:        usize = 64;
const QUALITY_THRESHOLD:  u8 = 50;

// cm/s @ 1 m -> urad/s
const FLOW_SCALE: i32 = 10_000;

#[embassy_executor::task]
pub async fn flow_task(
    uart: Peri<'static, peripherals::UART4>,
    rx:   Peri<'static, peripherals::PC11>,
    dma:  Peri<'static, peripherals::DMA1_CH2>,
    irqs: Irqs,
) {
    let mut cfg = Config::default();
    cfg.baudrate = 115_200;

    let rx = UartRx::new(uart, rx, dma, irqs, cfg)
        .expect("UART4 (optical flow) init failed");

    // Ring-buffered DMA RX so frames aren't dropped between reads.
    // 256 B ~ 22 ms of slack @ 115200 baud (50 Hz x 27 B frames).
    let mut rx_ring = [0u8; 256];
    let mut rx = rx.into_ring_buffered(&mut rx_ring);

    info!("Flow: MTF-02P on UART4 @ 115200 baud (MICOLINK)");

    let mut byte    = [0u8; 1];
    let mut hdr     = [0u8; 5];            // dev_id, sys_id, msg_id, seq, len
    let mut payload = [0u8; MAX_PAYLOAD];
    let mut cksum   = [0u8; 1];
    // Bench readout: ~2 Hz at the 50 Hz frame rate (nucleo-vcp builds only).
    #[cfg(feature = "nucleo-vcp")]
    let mut log_ctr: u32 = 0;

    // Silence detector: counts 500 ms sync timeouts (reset by ANY received
    // byte) so a dead line is loudly distinguishable from wrong-protocol
    // chatter (which shows up as checksum mismatches instead).
    let mut silent_ticks: u32 = 0;

    loop {
        // sync on header byte
        match with_timeout(Duration::from_millis(500), super::read_exact_ring(&mut rx, &mut byte)).await {
            Ok(Ok(())) if byte[0] == MICOLINK_HEAD => { silent_ticks = 0; }
            Ok(Ok(())) => { silent_ticks = 0; continue; } // byte arrived, not a header — resync
            _ => {
                silent_ticks += 1;
                if silent_ticks % 10 == 0 { // every ~5 s of dead air
                    warn!("Flow: UART4 SILENT ({=u32}s) — check module power, TX→PC11, jumpers",
                          silent_ticks / 2);
                }
                continue;
            }
        }

        // dev_id / sys_id / msg_id / seq / len
        if with_timeout(Duration::from_millis(20), super::read_exact_ring(&mut rx, &mut hdr)).await != Ok(Ok(())) {
            continue;
        }
        let msg_id = hdr[2];
        let len    = hdr[4] as usize;
        if len > MAX_PAYLOAD {
            continue; // bogus length from a false sync - resync
        }

        // payload + checksum
        if with_timeout(Duration::from_millis(20), super::read_exact_ring(&mut rx, &mut payload[..len])).await != Ok(Ok(())) {
            continue;
        }
        if with_timeout(Duration::from_millis(10), super::read_exact_ring(&mut rx, &mut cksum)).await != Ok(Ok(())) {
            continue;
        }

        // checksum = wrapping sum of 0xEF + 5 header bytes + payload
        let sum = hdr.iter().chain(payload[..len].iter())
            .fold(MICOLINK_HEAD, |acc, &b| acc.wrapping_add(b));
        if sum != cksum[0] {
            warn!("Flow: checksum mismatch — dropped frame");
            continue;
        }

        if msg_id != MSG_ID_RANGE_FLOW || len != RANGE_FLOW_LEN {
            continue; // other MICOLINK traffic - ignore
        }

        let distance_mm = u32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);
        let flow_x      = i16::from_le_bytes([payload[12], payload[13]]) as i32;
        let flow_y      = i16::from_le_bytes([payload[14], payload[15]]) as i32;
        let quality     = payload[16];

        let height_mm: i32 = if distance_mm == 0 { -1 } else { distance_mm as i32 };
        let valid = quality >= QUALITY_THRESHOLD && height_mm > 0;

        *STATE.flow.lock().await = FlowData {
            quality,
            vel_x_mrad_s: flow_x * FLOW_SCALE,
            vel_y_mrad_s: flow_y * FLOW_SCALE,
            height_mm,
            valid,
            stamp_ms:     crate::types::stamp_now_ms(),
        };

        // Bench bring-up readout — polarity check: at fixed height, slide the
        // drone FORWARD → vx must go POSITIVE; slide RIGHT → vy POSITIVE.
        // (Raw MICOLINK units: cm/s at 1 m height.)
        #[cfg(feature = "nucleo-vcp")]
        {
            log_ctr += 1;
            if log_ctr % 25 == 0 {
                info!("FLOW  h={=i32}mm  q={=u8}  vx={=i32}  vy={=i32}  valid={=bool}",
                      height_mm, quality, flow_x, flow_y, valid);
            }
        }
    }
}
