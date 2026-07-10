//! u-blox M10 GPS (SEQURE M10-18) - UBX NAV-PVT parser on USART1 (PA10 RX, PA9 TX).
//! Configured via UBX-CFG-VALSET (NAV-PVT on UART1, 5 Hz); writes STATE.gps_fix.
//! Ships at 115200 baud (vendor spec, confirmed 2026-07-09); generic M10 default
//! is 38400 - try that if the module is silent.

use defmt::{info, warn};
use embassy_stm32::{
    peripherals,
    usart::{Config, Uart},
    Peri,
};
use embassy_time::{Duration, Timer};

use crate::{types::GpsFix, Irqs, STATE};

// UBX configuration frame (checksum pre-computed)

/// UBX-CFG-VALSET (M10): enable NAV-PVT output (1 per nav epoch) on UART1.
/// Checksum pre-computed over class+id+len+payload.
const CFG_VALSET_NAV_PVT: &[u8] = &[
    0xB5, 0x62,              // sync chars
    0x06, 0x8A,              // class=CFG, id=VALSET
    0x09, 0x00,              // length = 9
    0x00,                    // version
    0x01,                    // layers: RAM (takes effect immediately)
    0x00, 0x00,              // reserved
    0x07, 0x00, 0x91, 0x20, // key: CFG-MSGOUT-UBX_NAV_PVT_UART1
    0x01,                    // value: 1 per nav epoch
    0x53, 0x48,              // CK_A, CK_B
];

/// UBX-CFG-VALSET (M10): 5 Hz navigation rate (CFG-RATE-MEAS = 200 ms).
/// Position-hold needs >=5 Hz - the estimator gains and PosPid assume it.
const CFG_VALSET_RATE_5HZ: &[u8] = &[
    0xB5, 0x62,              // sync chars
    0x06, 0x8A,              // class=CFG, id=VALSET
    0x0A, 0x00,              // length = 10
    0x00,                    // version
    0x01,                    // layers: RAM
    0x00, 0x00,              // reserved
    0x01, 0x00, 0x21, 0x30, // key: CFG-RATE-MEAS (U2, ms)
    0xC8, 0x00,              // value: 200 ms -> 5 Hz
    0xB5, 0x81,              // CK_A, CK_B
];

// Helpers

/// Fletcher-8 checksum over a sequence of byte slices (concatenated).
fn ubx_cksum_parts(a_slice: &[u8], b_slice: &[u8]) -> (u8, u8) {
    let (mut a, mut b) = (0u8, 0u8);
    for &byte in a_slice.iter().chain(b_slice.iter()) {
        a = a.wrapping_add(byte);
        b = b.wrapping_add(a);
    }
    (a, b)
}

fn parse_pvt(payload: &[u8; 92]) -> GpsFix {
    let fix_type = payload[20];
    let flags    = payload[21];
    let lon  = i32::from_le_bytes(payload[24..28].try_into().unwrap());
    let lat  = i32::from_le_bytes(payload[28..32].try_into().unwrap());
    let hmsl = i32::from_le_bytes(payload[36..40].try_into().unwrap());
    let hacc = u32::from_le_bytes(payload[40..44].try_into().unwrap());
    let vn   = i32::from_le_bytes(payload[48..52].try_into().unwrap());
    let ve   = i32::from_le_bytes(payload[52..56].try_into().unwrap());
    let vd   = i32::from_le_bytes(payload[56..60].try_into().unwrap());
    GpsFix {
        lat_deg:  lat as f64 * 1e-7,
        lon_deg:  lon as f64 * 1e-7,
        alt_m:    hmsl as f32 * 1e-3,
        vel_n_ms: vn  as f32 * 1e-3,
        vel_e_ms: ve  as f32 * 1e-3,
        vel_d_ms: vd  as f32 * 1e-3,
        hacc_m:   hacc as f32 * 1e-3,
        fix_ok:   (flags & 0x01) != 0 && fix_type >= 3,
        fix_type,
        stamp_ms: crate::types::stamp_now_ms(),
    }
}

// Task

#[embassy_executor::task]
pub async fn gps_task(
    uart_peri: Peri<'static, peripherals::USART1>,
    rx_pin:    Peri<'static, peripherals::PA10>,
    tx_pin:    Peri<'static, peripherals::PA9>,
    tx_dma:    Peri<'static, peripherals::DMA2_CH6>,
    rx_dma:    Peri<'static, peripherals::DMA2_CH5>,
    irqs:      Irqs,
) {
    let mut cfg = Config::default();
    // SEQURE M10-18 vendor default (spec sheet). Generic u-blox M10 modules
    // ship at 38400 - fall back to that if this module is silent.
    cfg.baudrate = 115200;

    let uart = Uart::new(uart_peri, rx_pin, tx_pin, tx_dma, rx_dma, irqs, cfg)
        .expect("USART1 (GPS) init failed");
    let (mut tx, rx) = uart.split();

    // Give the module time to boot, then configure via CFG-VALSET (M10 API):
    // NAV-PVT output on UART1, then a 5 Hz nav rate.
    Timer::after(Duration::from_millis(200)).await;
    tx.write(CFG_VALSET_NAV_PVT).await.ok();
    Timer::after(Duration::from_millis(50)).await;
    tx.write(CFG_VALSET_RATE_5HZ).await.ok();

    // Ring-buffered DMA RX - one-shot byte reads lose bytes under load and
    // corrupt UBX frames. 512 B is ~44 ms of slack at 115200 baud.
    let mut rx_ring = [0u8; 512];
    let mut rx = rx.into_ring_buffered(&mut rx_ring);

    info!("GPS task started — UBX NAV-PVT @ 5 Hz, 115200 baud on USART1 (M10-18)");

    let mut byte    = [0u8; 1];
    let mut hdr     = [0u8; 4];   // [class, id, len_l, len_h]
    let mut payload = [0u8; 92];
    let mut ckbuf   = [0u8; 2];
    // Fix logging: edge + every 5 s (per-fix logging floods RTT at 5 Hz).
    let mut had_fix   = false;
    let mut last_log  = embassy_time::Instant::now();

    loop {
        // sync char 1
        if super::read_exact_ring(&mut rx, &mut byte).await.is_err() { continue; }
        if byte[0] != 0xB5 { continue; }

        // sync char 2
        if super::read_exact_ring(&mut rx, &mut byte).await.is_err() { continue; }
        if byte[0] != 0x62 { continue; }

        // class / id / length (4 bytes)
        if super::read_exact_ring(&mut rx, &mut hdr).await.is_err() { continue; }
        let class = hdr[0];
        let id    = hdr[1];
        let len   = u16::from_le_bytes([hdr[2], hdr[3]]) as usize;

        // Guard against a false sync match yielding a bogus length that would
        // stall the drain below for thousands of bytes - treat as noise, resync.
        const MAX_UBX_LEN: usize = 512;
        if len > MAX_UBX_LEN { continue; }

        // Drain and discard any message that isn't NAV-PVT (92 bytes)
        if class != 0x01 || id != 0x07 || len != 92 {
            for _ in 0..len.saturating_add(2) {
                if super::read_exact_ring(&mut rx, &mut byte).await.is_err() { break; }
            }
            continue;
        }

        // payload
        if super::read_exact_ring(&mut rx, &mut payload).await.is_err() { continue; }

        // checksum
        if super::read_exact_ring(&mut rx, &mut ckbuf).await.is_err() { continue; }
        let (ck_a, ck_b) = ubx_cksum_parts(&hdr, &payload);
        if ck_a != ckbuf[0] || ck_b != ckbuf[1] {
            warn!("GPS: checksum mismatch — dropped frame");
            continue;
        }

        let fix = parse_pvt(&payload);
        let num_sv = payload[23]; // numSV - satellites used in the nav solution
        *STATE.gps_fix.lock().await = fix;

        // Log on the no-fix->fix edge, then every 5 s. No-fix also logs at 5 s so
        // the bench can tell "parsing, waiting for sats" from dead wiring.
        if fix.fix_ok && (!had_fix || last_log.elapsed() > Duration::from_secs(5)) {
            info!("GPS 3D fix  lat={=f64}  lon={=f64}  alt={=f32}m  hacc={=f32}m  sats={=u8}",
                  fix.lat_deg, fix.lon_deg, fix.alt_m, fix.hacc_m, num_sv);
            last_log = embassy_time::Instant::now();
        } else if !fix.fix_ok && last_log.elapsed() > Duration::from_secs(5) {
            info!("GPS: NAV-PVT arriving (no fix yet) — fixType={=u8} sats={=u8}",
                  fix.fix_type, num_sv);
            last_log = embassy_time::Instant::now();
        }
        had_fix = fix.fix_ok;
    }
}
