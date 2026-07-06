//! u-blox M10 GPS (SEQURE M10-18) — UBX NAV-PVT parser on USART1 (PA10 RX, PA9 TX).
//!
//! The M10 chip replaced the M8N's CFG-PRT / CFG-MSG protocol with UBX-CFG-VALSET.
//! Default baud rate on M10 is 38400 (M8N was 9600).
//!
//! Startup sequence:
//!   1. UBX-CFG-VALSET — enable NAV-PVT output on UART1
//!      (module typically ships with NMEA enabled; we add UBX NAV-PVT on top)
//!   2. UBX-CFG-VALSET — CFG-RATE-MEAS = 200 ms → 5 Hz navigation rate
//!
//! Then loops reading NAV-PVT frames (92-byte payload) and writing STATE.gps_fix.
//! Non-NAV-PVT frames (NMEA or other UBX) are drained and discarded.
//!
//! If the module doesn't respond, verify the default baud rate with a logic
//! analyzer on PA10 — some M10 modules ship at 115200 instead of 38400.
//!
//! NAV-PVT payload offsets used here:
//!   byte 20      fixType  (3 = 3-D fix)
//!   byte 21      flags    (bit 0 = gnssFixOK)
//!   bytes 24–27  lon      (i32, 1e-7 °)
//!   bytes 28–31  lat      (i32, 1e-7 °)
//!   bytes 36–39  hMSL     (i32, mm above MSL)
//!   bytes 40–43  hAcc     (u32, mm)
//!   bytes 48–51  velN     (i32, mm/s)
//!   bytes 52–55  velE     (i32, mm/s)
//!   bytes 56–59  velD     (i32, mm/s)

use defmt::{info, warn};
use embassy_stm32::{
    peripherals,
    usart::{Config, Uart},
    Peri,
};
use embassy_time::{Duration, Timer};

use crate::{types::GpsFix, Irqs, STATE};

// ── UBX configuration frame (checksum pre-computed) ────────────────────────

/// UBX-CFG-VALSET (M10): enable NAV-PVT output (1 per nav epoch) on UART1.
///
/// Payload (9 bytes):
///   version=0x00, layers=0x01 (RAM), reserved=0x00 0x00,
///   key=0x20910007 (CFG-MSGOUT-UBX_NAV_PVT_UART1, type U1), value=0x01
///
/// Checksum covers class+id+len+payload: CK_A=0x53, CK_B=0x48.
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
/// Position-hold needs ≥5 Hz — the estimator gains and PosPid assume it.
///
/// Payload (10 bytes):
///   version=0x00, layers=0x01 (RAM), reserved=0x00 0x00,
///   key=0x30210001 (CFG-RATE-MEAS, type U2), value=200 (ms)
const CFG_VALSET_RATE_5HZ: &[u8] = &[
    0xB5, 0x62,              // sync chars
    0x06, 0x8A,              // class=CFG, id=VALSET
    0x0A, 0x00,              // length = 10
    0x00,                    // version
    0x01,                    // layers: RAM
    0x00, 0x00,              // reserved
    0x01, 0x00, 0x21, 0x30, // key: CFG-RATE-MEAS (U2, ms)
    0xC8, 0x00,              // value: 200 ms → 5 Hz
    0xB5, 0x81,              // CK_A, CK_B
];

// ── Helpers ─────────────────────────────────────────────────────────────────

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

// ── Task ─────────────────────────────────────────────────────────────────────

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
    cfg.baudrate = 38400; // u-blox M10 default (M8N was 9600)

    let uart = Uart::new(uart_peri, rx_pin, tx_pin, tx_dma, rx_dma, irqs, cfg)
        .expect("USART1 (GPS) init failed");
    let (mut tx, rx) = uart.split();

    // Give the module time to boot, then configure via CFG-VALSET (M10 API):
    // NAV-PVT output on UART1, then a 5 Hz nav rate.
    Timer::after(Duration::from_millis(200)).await;
    tx.write(CFG_VALSET_NAV_PVT).await.ok();
    Timer::after(Duration::from_millis(50)).await;
    tx.write(CFG_VALSET_RATE_5HZ).await.ok();

    // Ring-buffered DMA RX so the interleaved NMEA/UBX stream isn't dropped between
    // reads — one-shot byte reads lose bytes under load and corrupt UBX frames.
    // 512 B ≈ 130 ms of slack @ 38400 baud.
    let mut rx_ring = [0u8; 512];
    let mut rx = rx.into_ring_buffered(&mut rx_ring);

    info!("GPS task started — UBX NAV-PVT @ 5 Hz, 38400 baud on USART1 (M10)");

    let mut byte    = [0u8; 1];
    let mut hdr     = [0u8; 4];   // [class, id, len_l, len_h]
    let mut payload = [0u8; 92];
    let mut ckbuf   = [0u8; 2];
    // Fix logging: edge + every 5 s (per-fix logging floods RTT at 5 Hz).
    let mut had_fix   = false;
    let mut last_log  = embassy_time::Instant::now();

    loop {
        // ── sync char 1 ────────────────────────────────────────────────────
        if super::read_exact_ring(&mut rx, &mut byte).await.is_err() { continue; }
        if byte[0] != 0xB5 { continue; }

        // ── sync char 2 ────────────────────────────────────────────────────
        if super::read_exact_ring(&mut rx, &mut byte).await.is_err() { continue; }
        if byte[0] != 0x62 { continue; }

        // ── class / id / length (4 bytes) ──────────────────────────────────
        if super::read_exact_ring(&mut rx, &mut hdr).await.is_err() { continue; }
        let class = hdr[0];
        let id    = hdr[1];
        let len   = u16::from_le_bytes([hdr[2], hdr[3]]) as usize;

        // Guard against a false 0xB5 0x62 sync match yielding a bogus length: real UBX
        // messages here are small (NAV-PVT = 92). A huge len would make the drain below
        // block for thousands of bytes, stalling parsing — treat it as noise and resync.
        const MAX_UBX_LEN: usize = 512;
        if len > MAX_UBX_LEN { continue; }

        // Drain and discard any message that isn't NAV-PVT (92 bytes)
        if class != 0x01 || id != 0x07 || len != 92 {
            for _ in 0..len.saturating_add(2) {
                if super::read_exact_ring(&mut rx, &mut byte).await.is_err() { break; }
            }
            continue;
        }

        // ── payload ────────────────────────────────────────────────────────
        if super::read_exact_ring(&mut rx, &mut payload).await.is_err() { continue; }

        // ── checksum ───────────────────────────────────────────────────────
        if super::read_exact_ring(&mut rx, &mut ckbuf).await.is_err() { continue; }
        let (ck_a, ck_b) = ubx_cksum_parts(&hdr, &payload);
        if ck_a != ckbuf[0] || ck_b != ckbuf[1] {
            warn!("GPS: checksum mismatch — dropped frame");
            continue;
        }

        let fix = parse_pvt(&payload);
        *STATE.gps_fix.lock().await = fix;

        // Log on the no-fix→fix edge, then at most every 5 s.
        if fix.fix_ok && (!had_fix || last_log.elapsed() > Duration::from_secs(5)) {
            info!("GPS 3D fix  lat={=f64}  lon={=f64}  alt={=f32}m  hacc={=f32}m",
                  fix.lat_deg, fix.lon_deg, fix.alt_m, fix.hacc_m);
            last_log = embassy_time::Instant::now();
        }
        had_fix = fix.fix_ok;
    }
}
