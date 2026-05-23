//! u-blox M8N GPS — UBX NAV-PVT parser on USART1 (PA10 RX, PA9 TX, 9600 baud).
//!
//! Startup sequence:
//!   1. UBX-CFG-PRT  — switch UART1 to UBX-only I/O at 9600 baud
//!   2. UBX-CFG-MSG  — enable NAV-PVT at 1 Hz on UART1
//!
//! Then loops reading NAV-PVT frames (92-byte payload) and writing STATE.gps_fix.
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

// ── UBX configuration frames (checksums pre-computed) ──────────────────────

/// UBX-CFG-PRT: UART1, UBX-only in/out, 8N1, 9600 baud.
const CFG_PRT: &[u8] = &[
    0xB5, 0x62,                          // sync chars
    0x06, 0x00, 0x14, 0x00,             // class=CFG, id=PRT, len=20
    0x01,                                // portID = UART1
    0x00, 0x00, 0x00,                    // reserved0, txReady (disabled)
    0xC0, 0x08, 0x00, 0x00,             // mode: 8N1
    0x80, 0x25, 0x00, 0x00,             // baudRate: 9600 (0x2580)
    0x01, 0x00,                          // inProtoMask:  UBX only
    0x01, 0x00,                          // outProtoMask: UBX only
    0x00, 0x00,                          // flags
    0x00, 0x00,                          // reserved5
    0x8A, 0xEF,                          // CK_A, CK_B
];

/// UBX-CFG-MSG: enable NAV-PVT (class=0x01 id=0x07) at 1 Hz on UART1.
const CFG_MSG: &[u8] = &[
    0xB5, 0x62,
    0x06, 0x01, 0x08, 0x00,             // class=CFG, id=MSG, len=8
    0x01, 0x07,                          // msgClass=NAV, msgID=PVT
    0x00,                                // I2C rate (off)
    0x01,                                // UART1 rate: 1 per nav epoch
    0x00, 0x00, 0x00, 0x00,             // UART2, USB, SPI, reserved
    0x18, 0xE1,                          // CK_A, CK_B
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
    cfg.baudrate = 9600;

    let uart = Uart::new(uart_peri, rx_pin, tx_pin, tx_dma, rx_dma, irqs, cfg)
        .expect("USART1 (GPS) init failed");
    let (mut tx, mut rx) = uart.split();

    // Give the module time to boot, then send configuration
    Timer::after(Duration::from_millis(200)).await;
    tx.write(CFG_PRT).await.ok();
    Timer::after(Duration::from_millis(100)).await;
    tx.write(CFG_MSG).await.ok();

    info!("GPS task started — UBX NAV-PVT @ 9600 baud on USART1");

    let mut byte    = [0u8; 1];
    let mut hdr     = [0u8; 4];   // [class, id, len_l, len_h]
    let mut payload = [0u8; 92];
    let mut ckbuf   = [0u8; 2];

    loop {
        // ── sync char 1 ────────────────────────────────────────────────────
        if rx.read(&mut byte).await.is_err() { continue; }
        if byte[0] != 0xB5 { continue; }

        // ── sync char 2 ────────────────────────────────────────────────────
        if rx.read(&mut byte).await.is_err() { continue; }
        if byte[0] != 0x62 { continue; }

        // ── class / id / length (4 bytes) ──────────────────────────────────
        if rx.read(&mut hdr).await.is_err() { continue; }
        let class = hdr[0];
        let id    = hdr[1];
        let len   = u16::from_le_bytes([hdr[2], hdr[3]]) as usize;

        // Drain and discard any message that isn't NAV-PVT (92 bytes)
        if class != 0x01 || id != 0x07 || len != 92 {
            for _ in 0..len.saturating_add(2) {
                if rx.read(&mut byte).await.is_err() { break; }
            }
            continue;
        }

        // ── payload ────────────────────────────────────────────────────────
        if rx.read(&mut payload).await.is_err() { continue; }

        // ── checksum ───────────────────────────────────────────────────────
        if rx.read(&mut ckbuf).await.is_err() { continue; }
        let (ck_a, ck_b) = ubx_cksum_parts(&hdr, &payload);
        if ck_a != ckbuf[0] || ck_b != ckbuf[1] {
            warn!("GPS: checksum mismatch — dropped frame");
            continue;
        }

        let fix = parse_pvt(&payload);
        *STATE.gps_fix.lock().await = fix;

        if fix.fix_ok {
            info!("GPS 3D fix  lat={=f64}  lon={=f64}  alt={=f32}m  hacc={=f32}m",
                  fix.lat_deg, fix.lon_deg, fix.alt_m, fix.hacc_m);
        }
    }
}
