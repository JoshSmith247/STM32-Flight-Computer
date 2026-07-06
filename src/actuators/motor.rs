//! Motor output — DSHOT300 (default) via TIM3 burst DMA. `--features dshot600`
//! doubles the bit rate for ESCs that support it — the fitted HGLRC 60A V2
//! BLHeli_S ESC only parses DShot300, despite its listing claiming both.
//!
//! TIM3: 200 MHz timer clock, PSC=1 → 100 MHz, ARR=332 → 3.33 µs/bit (300 kHz).
//! DMA1 stream 4 writes 72 u16 values to TIM3_DMAR on each timer update event.
//! Each update event distributes 4 CCR values (one per motor) via TIM3's burst DMA,
//! advancing through 16 data bits + 2 reset slots = 18 timer periods per DSHOT frame.
//!
//! Buffer layout (interleaved, motor-major within each bit slot):
//!   buf[bit * 4 + motor_idx] = T1H or T0H for bits 0..16
//!   buf[64..72]              = 0  (reset / inter-frame gap — line stays LOW)
//!
//! DMA1_CH4 is reserved by this task via PAC; do not allocate it to any other peripheral.
//!
//! Pinout (TIM3 AF2):
//!   M1 → PB4 (TIM3 CH1)   M3 → PB0 (TIM3 CH3)
//!   M2 → PB5 (TIM3 CH2)   M4 → PB1 (TIM3 CH4)

use core::sync::atomic::{compiler_fence, Ordering};

use defmt::info;
use embassy_stm32::{pac, peripherals, Peri};
use embassy_time::{Duration, Instant, Ticker};
#[cfg(feature = "pin-test")]
use embassy_time::Timer;

use crate::state::{self, FlightState};
use crate::STATE;

// ── DSHOT timing constants (bit width shared; PSC below selects 300 vs 600) ──

#[cfg_attr(feature = "pin-test", allow(dead_code))] // pin-test builds skip motor_task
const ARR:  u16 = 332;  // TIM period = 333 ticks; at 100 MHz (PSC=1) → 3.33 µs/bit = DSHOT300
const T1H:  u16 = 250;  // 75%   duty — logic 1
const T0H:  u16 = 125;  // 37.5% duty — logic 0

const SLOTS:   usize = 18;             // 16 data bits + 2 reset
const MOTORS:  usize = 4;
const BUF_LEN: usize = SLOTS * MOTORS; // 72 u16 halfwords (DMA to 16-bit TIM3_DMAR)

// Idle floor while ARMED: DSHOT 0 stops the rotor entirely — a stopped rotor
// has zero attitude authority and BLHeli_S restart in-air is not instant.
// ⚠ Motors spin at this the moment the craft arms — props clear!
#[cfg_attr(feature = "pin-test", allow(dead_code))]
const MOTOR_IDLE: f32 = 0.04;

// ⚠ BENCH ONLY (PROPS OFF): with `--features bench-force`, motor_task holds this one
// motor at this throttle every cycle — a constant DSHOT stream on its pin for
// scope/logic-analyzer probing, with no command/arming/watchdog. NEVER build for flight.
#[cfg(feature = "bench-force")]
const BENCH_FORCE_MOTOR:    u8  = 1;   // 1..=4 (M1=PB4, M2=PB5, M3=PB0, M4=PB1)
#[cfg(feature = "bench-force")]
const BENCH_FORCE_THROTTLE: f32 = 0.10;

// ⚠ BENCH ONLY (PROPS OFF): with `--features spin-all`, motor_task arms with continuous
// zeros for SPIN_ALL_ARM_SECS, then holds ALL FOUR motors at SPIN_ALL_THROTTLE so every
// motor spins together — no GCS script, no arming switch. NEVER build for flight.
#[cfg(feature = "spin-all")]
const SPIN_ALL_ARM_SECS:  u64 = 5;    // continuous-zero arming window before each throttle burst
#[cfg(feature = "spin-all")]
const SPIN_ALL_ON_SECS:   u64 = 5;    // throttle burst length before returning to zeros
#[cfg(feature = "spin-all")]
const SPIN_ALL_THROTTLE:  f32 = 0.30; // must exceed BLHeli_S startup/low-RPM threshold

// DMAMUX1 request ID for TIM3_UP (RM0468 Table 105) — verified on hardware 2026-06-20.
// Wrong value = DMA never fires = motors silently dead.
#[cfg_attr(feature = "pin-test", allow(dead_code))]
const DMAMUX_TIM3_UP: u8 = 27;

// ── DSHOT frame encoding ─────────────────────────────────────────────────────

fn throttle_to_dshot(t: f32) -> u16 {
    let t = t.clamp(0.0, 1.0);
    if t <= 0.0 { 0 } else { (48.0 + t * (2047.0 - 48.0)) as u16 }
}

fn encode_dshot(thr: u16) -> u16 {
    let payload = thr << 1; // telemetry request bit = 0
    let csum = payload ^ (payload >> 4) ^ (payload >> 8);
    // Bidirectional DSHOT inverts the checksum — that inversion is how the ESC
    // distinguishes the protocols. ⚠ Mutually exclusive on the wire with plain DSHOT.
    #[cfg(not(feature = "dshot-bidir"))]
    let crc = csum & 0x0F;
    #[cfg(feature = "dshot-bidir")]
    let crc = (!csum) & 0x0F;
    (payload << 4) | crc
}

// Buffer is u16: the DMA does halfword transfers to TIM3's 16-bit DMAR/CCR registers.
fn fill_buf(buf: &mut [u16; BUF_LEN], motors: [f32; MOTORS]) {
    let frames: [u16; MOTORS] =
        core::array::from_fn(|i| encode_dshot(throttle_to_dshot(motors[i])));

    for bit in 0..16 {
        for (m, &frame) in frames.iter().enumerate() {
            buf[bit * MOTORS + m] = if (frame >> (15 - bit)) & 1 == 1 { T1H } else { T0H };
        }
    }
    for slot in 16..SLOTS {
        for m in 0..MOTORS { buf[slot * MOTORS + m] = 0; }
    }
}

// ── Shared TX-path stats (#4: surfaced to the GCS over MAVLink) ───────────────

// Snapshot of the DSHOT health counters, published at 1 Hz by motor_task and read by
// telemetry_task, which emits them as a STATUSTEXT so they're visible in the GCS even
// when there's no probe-rs/RTT link on the bench (e.g. testing through the Pi serial).
#[cfg(feature = "dshot-debug")]
pub mod debug_stats {
    use core::sync::atomic::{AtomicU32, Ordering};

    pub static FRAMES:   AtomicU32 = AtomicU32::new(0); // 500 Hz ticks
    pub static TC:       AtomicU32 = AtomicU32::new(0); // transfer-complete seen
    pub static SKIPPED:  AtomicU32 = AtomicU32::new(0); // dshot_send bailed (DMA busy)
    pub static STUCK_EN: AtomicU32 = AtomicU32::new(0); // DMA wedged ≥ 2 ms
    pub static ERRORS:   AtomicU32 = AtomicU32::new(0); // teif + feif + dmeif
    pub static PIN_MASK: AtomicU32 = AtomicU32::new(0); // last pin-toggle probe (bits 4,5,0,1)

    #[allow(clippy::too_many_arguments)]
    pub fn publish(frames: u32, tc: u32, skipped: u32, stuck_en: u32, errors: u32, pin_mask: u8) {
        FRAMES.store(frames, Ordering::Relaxed);
        TC.store(tc, Ordering::Relaxed);
        SKIPPED.store(skipped, Ordering::Relaxed);
        STUCK_EN.store(stuck_en, Ordering::Relaxed);
        ERRORS.store(errors, Ordering::Relaxed);
        PIN_MASK.store(pin_mask as u32, Ordering::Relaxed);
    }
}

// ── Bidirectional DSHOT eRPM decode (#5: `--features dshot-bidir`) ─────────────
//
// ⚠ STATUS: the encode side (inverted CRC, above) and the decode logic below are
// implemented and self-consistent, but the on-the-wire EDGE CAPTURE is a hardware-
// timing-critical step that MUST be validated on the bench with a scope before the
// eRPM numbers can be trusted — see capture_erpm_raw(). Until then this path proves
// the protocol math, not a live RPM reading. Mutually exclusive with plain DSHOT.

// Motor magnet-pole count — eRPM = mechanical RPM × poles/2. Tune to the fitted motor
// (e.g. 14 for a typical 2207). Only used to convert the decoded eRPM to mechanical RPM.
#[cfg(feature = "dshot-bidir")]
const MOTOR_POLES: u32 = 14;

// 5-bit GCR → 4-bit nibble. 0xFF marks an invalid quintet (line/decode error).
#[cfg(feature = "dshot-bidir")]
const GCR_LUT: [u8; 32] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0x9,  0xA,  0xB,  0xFF, 0xD,  0xE,  0xF,
    0xFF, 0xFF, 0x2,  0x3,  0xFF, 0x5,  0x6,  0x7,
    0xFF, 0x0,  0x8,  0x1,  0xFF, 0x4,  0xC,  0xFF,
];

/// Decode a captured 20-bit bidirectional-DSHOT response into eRPM.
/// Returns `None` on any invalid quintet or bad telemetry checksum.
/// `poles` is the motor's magnet-pole count (eRPM = mechanical RPM × poles/2).
#[cfg(feature = "dshot-bidir")]
fn decode_erpm(raw20: u32, poles: u32) -> Option<u32> {
    // 1. Undo the line transition-encoding: each bit XORs with the one above it.
    let gcr = raw20 ^ (raw20 >> 1);
    // 2. Four 5-bit quintets → four nibbles → 16-bit frame.
    let mut frame: u16 = 0;
    for q in (0..20).step_by(5).rev() {
        let nib = GCR_LUT[((gcr >> q) & 0x1F) as usize];
        if nib == 0xFF { return None; }
        frame = (frame << 4) | nib as u16;
    }
    // 3. frame = eeem_mmmm_mmmm_cccc: 3-bit exp, 9-bit mantissa, 4-bit checksum.
    let csum = (frame ^ (frame >> 4) ^ (frame >> 8)) & 0x0F;
    if csum != 0x0F { return None; } // bidir telemetry checksum is the inverted-nibble XOR
    let value = frame >> 4;          // 12-bit: eee_mmmmmmmmm
    let period_us = ((value & 0x1FF) as u32) << ((value >> 9) & 0x07);
    if period_us == 0 { return None; }
    // eRPM = 60 s / period; mechanical RPM = eRPM / (poles/2).
    Some(60_000_000 / period_us * 2 / poles.max(2))
}

// ── DMA buffer ───────────────────────────────────────────────────────────────

// AXISRAM (0x2400_0000) is DMA-accessible on STM32H723; the linker places
// all statics there, so no special link section is needed.
static mut DSHOT_BUF: [u16; BUF_LEN] = [0u16; BUF_LEN];

// ── Hardware init ────────────────────────────────────────────────────────────

#[cfg_attr(feature = "pin-test", allow(dead_code))]
unsafe fn dshot_init() {
    use pac::gpio::vals::{Moder, Ospeedr, Ot};
    use pac::timer::vals::OcmGp;
    use pac::dma::vals::{Burst, Dir, Dmdis, Pl, Size};

    // Enable peripheral clocks
    pac::RCC.ahb4enr().modify(|w| w.set_gpioben(true));
    pac::RCC.apb1lenr().modify(|w| w.set_tim3en(true));
    pac::RCC.ahb1enr().modify(|w| w.set_dma1en(true));
    cortex_m::asm::dsb();

    // Configure PB0, PB1, PB4, PB5 → TIM3 AF2
    // All pins ≤ 7 so AFRL = afr(0)
    let gp = pac::GPIOB;
    for &pin in &[0usize, 1, 4, 5] {
        gp.moder().modify(|w| w.set_moder(pin, Moder::Alternate));
        gp.ospeedr().modify(|w| w.set_ospeedr(pin, Ospeedr::VeryHighSpeed));
        gp.otyper().modify(|w| w.set_ot(pin, Ot::PushPull));
        gp.afr(0).modify(|w| w.set_afr(pin, 2)); // AF2 = TIM3
    }

    // DSHOT bit rate: only the prescaler differs between 300 and 600 — duty
    // ratios are speed-independent, so ARR/T1H/T0H are shared.
    let tim = pac::TIM3;
    #[cfg(not(feature = "dshot600"))]
    tim.psc().write(|w| *w = 1u16);              // prescaler = 1 → 100 MHz → 300 kbit/s
    #[cfg(feature = "dshot600")]
    tim.psc().write(|w| *w = 0u16);              // prescaler = 0 → 200 MHz → 600 kbit/s
    tim.arr().write(|w| w.set_arr(ARR));          // period = 333 ticks

    // PWM mode 1 with output preload on all 4 channels.
    // ccmr_output(0) covers CH1 (n=0) and CH2 (n=1).
    // ccmr_output(1) covers CH3 (n=0) and CH4 (n=1).
    tim.ccmr_output(0).write(|w| {
        w.set_ocm(0, OcmGp::PwmMode1); w.set_ocpe(0, true);
        w.set_ocm(1, OcmGp::PwmMode1); w.set_ocpe(1, true);
    });
    tim.ccmr_output(1).write(|w| {
        w.set_ocm(0, OcmGp::PwmMode1); w.set_ocpe(0, true);
        w.set_ocm(1, OcmGp::PwmMode1); w.set_ocpe(1, true);
    });

    // Enable CC outputs; CCR=0 → line LOW while DMA is idle
    tim.ccer().write(|w| {
        w.set_cce(0, true); w.set_cce(1, true);
        w.set_cce(2, true); w.set_cce(3, true);
    });
    for ch in 0..4usize { tim.ccr(ch).write(|w| w.set_ccr(0)); }

    // Burst DMA: DBA=13 (CCR1 is at TIM base+0x34, offset 13 registers from CR1),
    // DBL=3 (burst length = 4 registers: CCR1→CCR4).
    tim.dcr().write(|w| { w.set_dba(13); w.set_dbl(3); });

    // Enable update → DMA request
    tim.dier().modify(|w| w.set_ude(true));

    // Start timer; generate update event to load preload registers into active
    tim.cr1().modify(|w| { w.set_arpe(true); w.set_cen(true); });
    tim.egr().write(|w| w.set_ug(true));

    // Route DMAMUX1 channel 4 to TIM3_UP
    pac::DMAMUX1.ccr(4).write(|w| w.set_dmareq_id(DMAMUX_TIM3_UP));

    // Configure DMA1 stream 4
    let s = pac::DMA1.st(4);
    s.cr().modify(|w| w.set_en(false));
    // Bounded: never hard-spin forever — a stuck DMA must not wedge the (single-
    // threaded) executor and take the whole flight computer down with it.
    let mut guard = 0u32;
    while s.cr().read().en() {
        core::hint::spin_loop();
        guard += 1;
        if guard > 1_000_000 { break; }
    }

    // Clear all stream-4 flags via the high-register IFCR (streams 4-7 are in ifcr(1))
    // Stream 4 is at local index 0 within that register.
    pac::DMA1.ifcr(1).write(|w| {
        w.set_tcif(0, true); w.set_htif(0, true);
        w.set_teif(0, true); w.set_dmeif(0, true);
        w.set_feif(0, true);
    });

    s.par().write(|w| *w = pac::TIM3.dmar().as_ptr() as u32); // fixed peripheral address (from PAC)

    // TIM_DMAR delivery constraints: NO burst (TIM3's DCR.DBL=3 does the CCR1..CCR4
    // fan-out itself by re-asserting the request), HALFWORD transfers (DMAR is a
    // 16-bit register — 32-bit writes corrupt the fan-out), DIRECT mode (a FIFO buys
    // nothing for single same-size transfers and only adds a FEIF failure mode).
    s.fcr().write(|w| {
        w.set_dmdis(Dmdis::Enabled); // direct mode — no FIFO
    });

    s.cr().write(|w| {
        w.set_dir(Dir::MemoryToPeripheral);
        w.set_minc(true);              // advance through DSHOT_BUF
        w.set_pinc(false);             // DMAR address is fixed
        w.set_msize(Size::Bits16);     // halfword: TIM3 DMAR/CCR are 16-bit
        w.set_psize(Size::Bits16);
        w.set_mburst(Burst::Single);   // NO burst — timer's DBL fan-out re-requests per CCR
        w.set_pburst(Burst::Single);
        w.set_pl(Pl::VeryHigh);
        w.set_circ(false);             // one-shot per frame
        w.set_en(false);
    });
}

/// Queue one DSHOT frame via DMA. Returns `false` (frame skipped) if the
/// previous DMA is still enabled — never busy-wait here: a stuck DMA must not
/// hog the single-threaded executor and starve the watchdog pet. A sustained
/// run of `false` means the DMA is wedged and the pins get NO new frames.
unsafe fn dshot_send() -> bool {
    let s = pac::DMA1.st(4);

    if s.cr().read().en() {
        return false;
    }

    pac::DMA1.ifcr(1).write(|w| {
        w.set_tcif(0, true); w.set_htif(0, true);
        w.set_teif(0, true); w.set_dmeif(0, true);
        w.set_feif(0, true);
    });

    s.m0ar().write(|w| *w = core::ptr::addr_of!(DSHOT_BUF) as u32);
    s.ndtr().write(|w| w.set_ndt(BUF_LEN as u16));
    s.cr().modify(|w| w.set_en(true)); // timer update event fires DMA
    true
}

// ── DSHOT TX instrumentation (`--features dshot-debug`) ───────────────────────

/// Running tallies of the DSHOT TX path, logged at 1 Hz by motor_task. Cumulative
/// since boot. Healthy steady state: `frames` and `tc` climb together, everything
/// else flat at 0.
#[cfg(feature = "dshot-debug")]
#[derive(Default)]
struct DshotDbg {
    frames:     u32, // 500 Hz ticks observed
    tc:         u32, // previous frame's transfer-complete flag was set (good)
    skipped:    u32, // dshot_send() bailed: DMA still enabled (no frame queued)
    stuck_en:   u32, // DMA still EN at the next tick → wedged ≥ 2 ms
    short_xfer: u32, // DMA disabled but NDTR ≠ 0 → frame truncated
    teif:       u32, // transfer error
    feif:       u32, // FIFO error
    dmeif:      u32, // direct-mode error
    prev_active: bool, // any motor commanded non-zero last tick (for edge detection)
    last_pin_mask: u8, // most recent pin-toggle probe result
    // Loop-cadence tracking: exposes bursty executor scheduling. Reset each 1 Hz window.
    last_loop:  Option<Instant>, // timestamp of previous loop iteration
    max_gap_us: u32,             // largest gap between iterations (steady 500 Hz ≈ 2000)
    long_gaps:  u32,             // gaps > 5 ms (executor slept past ≥2 ticks)
    short_gaps: u32,             // gaps < 1 ms (Ticker firing catch-up frames back-to-back)
}

/// One-shot readback of the GPIO AF / TIM3 / DMA / DMAMUX configuration after init,
/// so a dead bench can be triaged from defmt alone: if any pin isn't AF2, the timer
/// isn't counting, or the DMAMUX request is wrong, the firmware half is the bottleneck.
#[cfg(feature = "dshot-debug")]
unsafe fn dshot_debug_dump() {
    use pac::gpio::vals::Moder;

    let gp = pac::GPIOB;
    for &pin in &[4usize, 5, 0, 1] {
        let is_alt = gp.moder().read().moder(pin) == Moder::Alternate;
        let af     = gp.afr(0).read().afr(pin);
        info!("DSHOT dbg: PB{=usize} alt={=bool} af={=u8} (want alt=true af=2)", pin, is_alt, af);
    }

    let tim = pac::TIM3;
    let c0 = tim.cnt().read().cnt();
    for _ in 0..1000 { core::hint::spin_loop(); } // ~5 µs at 200 MHz — CNT must wrap
    let c1 = tim.cnt().read().cnt();
    info!("DSHOT dbg: TIM3 cen={=bool} ude={=bool} arr={=u16} cnt_a={=u16} cnt_b={=u16} (cnt must differ)",
          tim.cr1().read().cen(), tim.dier().read().ude(), tim.arr().read().arr(), c0, c1);

    info!("DSHOT dbg: DMAMUX ccr4 req_id={=u8} (want {=u8}); DMA1.st4 en={=bool}",
          pac::DMAMUX1.ccr(4).read().dmareq_id(), DMAMUX_TIM3_UP, pac::DMA1.st(4).cr().read().en());

    // Confirm the DMA stream config is actually what we set (don't infer it). par must equal
    // TIM3.dmar() (0x4000_044C). dmdis=1 means FIFO mode, dmdis=0 means DIRECT mode (no FIFO,
    // so feif is structurally impossible). msize/psize must be 0 (=16-bit) for the 16-bit DMAR.
    // dcr: dba=13 (CCR1), dbl=3 (4 registers CCR1..4).
    {
        let st  = pac::DMA1.st(4);
        let cr  = st.cr().read();
        let par = st.par().read();
        let fcr = st.fcr().read();
        let dcr = pac::TIM3.dcr().read();
        info!("DSHOT dbg: par=0x{=u32:08X} (want 0x{=u32:08X}) dmdis={=u8} msize={=u8} psize={=u8} dba={=u8} dbl={=u8} (dmdis0=direct, size0=16bit, dba13 dbl3)",
              par, pac::TIM3.dmar().as_ptr() as u32, fcr.dmdis() as u8,
              cr.msize() as u8, cr.psize() as u8, dcr.dba(), dcr.dbl());
    }

    // #2: confirm the DMA reads the RIGHT memory. m0ar must equal &DSHOT_BUF AND sit in
    // AXI SRAM (0x24xx_xxxx). The classic H7 trap: a buffer in DTCM (0x2000_xxxx) is
    // INVISIBLE to DMA1/DMA2 — every other stage looks healthy while the DMA reads
    // nothing. memory.x places statics in AXISRAM, so this should read ok=true.
    let m0ar    = pac::DMA1.st(4).m0ar().read();
    let want    = core::ptr::addr_of!(DSHOT_BUF) as u32;
    let in_axi  = (m0ar & 0xFF00_0000) == 0x2400_0000;
    info!("DSHOT dbg: m0ar=0x{=u32:08X} buf=0x{=u32:08X} match={=bool} in_axisram={=bool} (DMA can't see DTCM 0x2000_xxxx)",
          m0ar, want, m0ar == want, in_axi);

    // BISECTION: is the fault the TIMER OUTPUT stage or the DMA delivery? Bypass the DMA
    // entirely — read back the CC-output enables, then force a static 50 %-duty PWM by
    // writing CCR directly and probe the pins. Runs once at boot before the loop drives
    // the DMA, so nothing competes for the CCRs.
    //   • static mask ≠ 0  → timer + CCER + pins are fine; the DMA burst delivery is the bug.
    //   • static mask = 0  → the timer OUTPUT stage itself isn't driving the pads (CCER/
    //                        CCMR), and the DMA is a red herring.
    let ccer = tim.ccer().read();
    info!("DSHOT dbg: TIM3 CCER cc1e={=bool} cc2e={=bool} cc3e={=bool} cc4e={=bool}",
          ccer.cce(0), ccer.cce(1), ccer.cce(2), ccer.cce(3));
    for ch in 0..4usize { tim.ccr(ch).write(|w| w.set_ccr(ARR / 2)); } // ~50 % duty
    tim.egr().write(|w| w.set_ug(true));                               // load preload → active
    let static_mask = dshot_probe_pins(50);
    let idr_raw     = pac::GPIOB.idr().read().0 as u16;
    info!("DSHOT dbg: STATIC-PWM no-DMA (CCR={=u16}) mask=0b{=u8:04b} idr=0b{=u16:016b} — toggle here = output stage OK → DMA is the fault",
          ARR / 2, static_mask, idr_raw);

    // TIMING: fire ONE DMA frame uninterrupted (executor not running yet) and count CPU
    // iterations to completion. The timer free-runs in hardware regardless of this loop,
    // so a healthy DMA should drain 72 transfers in ~30 µs ≈ a few thousand iters with
    // ndtr_end=0, tcif=true. A timeout (iters maxed, ndtr_end≈65) means the DMA genuinely
    // can't clock a frame even in isolation → pure DMA/timer config, not load/contention.
    fill_buf(&mut *(&raw mut DSHOT_BUF), [0.10f32; MOTORS]);
    compiler_fence(Ordering::SeqCst);
    let st = pac::DMA1.st(4);
    let mut wait = 0u32;
    while st.cr().read().en() && wait < 2_000_000 { wait += 1; core::hint::spin_loop(); }
    pac::DMA1.ifcr(1).write(|w| {
        w.set_tcif(0, true); w.set_htif(0, true);
        w.set_teif(0, true); w.set_dmeif(0, true); w.set_feif(0, true);
    });
    st.m0ar().write(|w| *w = core::ptr::addr_of!(DSHOT_BUF) as u32);
    st.ndtr().write(|w| w.set_ndt(BUF_LEN as u16));
    st.cr().modify(|w| w.set_en(true));
    let mut iters = 0u32;
    while st.cr().read().en() && iters < 5_000_000 { iters += 1; core::hint::spin_loop(); }
    let isr = pac::DMA1.isr(1).read();
    info!("DSHOT dbg: TIMING one frame: iters={=u32} ndtr_end={=u16} tcif={=bool} feif={=bool} teif={=bool} dmeif={=bool} (few-k iters + ndtr0 = DMA healthy in isolation)",
          iters, st.ndtr().read().ndt(), isr.tcif(0), isr.feif(0), isr.teif(0), isr.dmeif(0));
}

/// #3 — software "poor-man's logic probe". The pins stay timer-driven (AF2); we only
/// READ. On STM32 the input register reflects the live pad level even for an AF output,
/// so by sampling GPIOB.IDR in a tight loop for ~`window_us` and OR-ing the results we
/// learn which of PB4/PB5/PB0/PB1 actually toggled high — i.e. is the silicon physically
/// wiggling the pad, one stage past "config looks right". Returns a mask in bit order
/// (PB4, PB5, PB0, PB1) → (b0, b1, b2, b3). Bounded and non-destructive; keep the window
/// short (≤100 µs) so it can't starve the single-threaded executor / watchdog.
#[cfg(feature = "dshot-debug")]
fn dshot_probe_pins(window_us: u32) -> u8 {
    let idr = pac::GPIOB.idr();
    let mut seen: u16 = 0;
    // ~200 MHz core; ~12 reads/µs is plenty to catch a 600 kHz toggle over the window.
    let iters = window_us.saturating_mul(12);
    for _ in 0..iters {
        seen |= idr.read().0 as u16;
        core::hint::spin_loop();
    }
    let bit = |p: usize| ((seen >> p) & 1) as u8;
    bit(4) | (bit(5) << 1) | (bit(0) << 2) | (bit(1) << 3)
}

/// #5 — capture the ESC's bidirectional-DSHOT eRPM reply on one motor pin.
/// ⚠ SCAFFOLD — NOT bench-validated. After a frame the ESC drives ~21 GCR bits back
/// onto the (now-released) line ≈30 µs later at 5/4 the bit rate. A correct capture
/// reconfigures the channel to input-capture + DMA and timestamps edges; doing that
/// reliably needs a scope to confirm timing on real hardware. This stub returns the
/// raw 20-bit word once that capture exists, to feed `decode_erpm`. Left explicit so it
/// can't masquerade as working telemetry.
#[cfg(feature = "dshot-bidir")]
fn capture_erpm_raw(_pin: usize) -> Option<u32> {
    // TODO(bench): switch CHx to TI input-capture, DMA the edge timestamps, reconstruct
    // the 20-bit GCR word from inter-edge timing, then `decode_erpm(raw, POLES)`.
    None
}

// ── Emergency stop (called from panic handler) ───────────────────────────────

/// Zero all motors and fire one DSHOT stop frame. Panic-handler safe: drains
/// any in-flight DMA (bounded), force-aborting a wedged stream, then sends —
/// dshot_send() bails while EN is set, so the drain must happen here.
pub(crate) unsafe fn emergency_stop() {
    let s = pac::DMA1.st(4);
    let mut guard = 0u32;
    while s.cr().read().en() && guard < 20_000 {
        guard += 1;
        core::hint::spin_loop();
    }
    if s.cr().read().en() {
        // Wedged: request abort (EN=0), then wait for it so the re-enable takes.
        s.cr().modify(|w| w.set_en(false));
        guard = 0;
        while s.cr().read().en() && guard < 20_000 {
            guard += 1;
            core::hint::spin_loop();
        }
    }

    fill_buf(&mut *(&raw mut DSHOT_BUF), [0.0f32; MOTORS]);
    compiler_fence(Ordering::SeqCst);
    dshot_send();
}

// ── Task ─────────────────────────────────────────────────────────────────────

/// ⚠ BENCH DIAGNOSTIC (`--features pin-test`): replaces motor_task. Drives the four motor
/// pins as plain push-pull GPIO, one HIGH at a time for 4 s, so a multimeter in DC-volts
/// mode can confirm each STM pin reaches the correct ESC signal pad. No DShot, no TIM3 —
/// the ESC only ever sees a static level (never a valid throttle), so no motor moves.
/// NOTE: PB0 (M3) is also the Nucleo green LED LD1, so that pin self-indicates.
#[cfg(feature = "pin-test")]
#[embassy_executor::task]
pub async fn pin_test_task(
    m1: Peri<'static, peripherals::PB4>,
    m2: Peri<'static, peripherals::PB5>,
    m3: Peri<'static, peripherals::PB0>,
    m4: Peri<'static, peripherals::PB1>,
) {
    use embassy_stm32::gpio::{Level, Output, Speed};

    let mut pins: [Output<'static>; 4] = [
        Output::new(m1, Level::Low, Speed::Low),
        Output::new(m2, Level::Low, Speed::Low),
        Output::new(m3, Level::Low, Speed::Low),
        Output::new(m4, Level::Low, Speed::Low),
    ];
    let names = ["M1 = PB4", "M2 = PB5", "M3 = PB0 (also LD1)", "M4 = PB1"];

    info!("PIN-TEST: driving each motor pin HIGH for 4 s in turn. Probe the matching ESC \
           signal pad with a DMM (DC volts) — 3.3 V there = that STM pin reaches that pad.");

    loop {
        for i in 0..4 {
            info!("PIN-TEST: {} HIGH (others LOW)", names[i]);
            pins[i].set_high();
            Timer::after(Duration::from_secs(4)).await;
            pins[i].set_low();
            Timer::after(Duration::from_millis(500)).await;
        }
    }
}

#[embassy_executor::task]
pub async fn motor_task(
    // Consumed to reserve peripherals; all hardware configured via PAC.
    _tim: Peri<'static, peripherals::TIM3>,
    _m1:  Peri<'static, peripherals::PB4>,
    _m2:  Peri<'static, peripherals::PB5>,
    _m3:  Peri<'static, peripherals::PB0>,
    _m4:  Peri<'static, peripherals::PB1>,
) {
    unsafe { dshot_init() };

    unsafe {
        fill_buf(&mut *(&raw mut DSHOT_BUF), [0.0f32; MOTORS]);
        compiler_fence(Ordering::SeqCst);
        dshot_send();
    }

    // ⚠ BENCH (`--features dshot-debug`): dump the TX-path config once. Must run AFTER
    // the first dshot_send — M0AR is unpopulated before it and misreports as a mismatch.
    #[cfg(feature = "dshot-debug")]
    unsafe { dshot_debug_dump() };

    // ESC arming window: 2 s of continuous zero frames at 500 Hz — BLHeli_S
    // treats a silent line as signal loss and won't arm.
    {
        let mut arm_ticker = Ticker::every(Duration::from_hz(500));
        for _ in 0..1000u32 {
            arm_ticker.next().await;
            unsafe {
                let st = pac::DMA1.st(4);
                if !st.cr().read().en() {
                    fill_buf(&mut *(&raw mut DSHOT_BUF), [0.0f32; MOTORS]);
                    compiler_fence(Ordering::SeqCst);
                    dshot_send();
                    // WFE clock-gating workaround: stay awake until the frame drains.
                    let mut guard = 0u32;
                    while st.cr().read().en() && guard < 20_000 {
                        guard += 1;
                        core::hint::spin_loop();
                    }
                }
            }
        }
    }

    #[cfg(not(feature = "dshot600"))]
    info!("Motors: DSHOT300 running (TIM3 + DMA1_CH4, PSC=1)");
    #[cfg(feature = "dshot600")]
    info!("Motors: DSHOT600 running (TIM3 + DMA1_CH4, PSC=0)");

    let mut ticker = Ticker::every(Duration::from_hz(500));

    #[cfg(feature = "dshot-debug")]
    let mut dbg = DshotDbg::default();

    // spin-all timing: the loop streams zeros until SPIN_ALL_ARM_SECS elapses (arming),
    // then throttles all four. spin_start is measured from here so the 500 Hz zero stream
    // gives the ESCs a clean, gap-free arming signal before any throttle.
    #[cfg(feature = "spin-all")]
    let spin_start = Instant::now();
    #[cfg(feature = "spin-all")]
    let mut spin_announced = false;

    loop {
        ticker.next().await;

        // ⚠ BENCH (`--features dshot-debug`): sample the DMA state left by the PREVIOUS
        // frame (it has had a full 2 ms tick to finish), then log cumulative health at
        // 1 Hz. Read at the top of the loop, before dshot_send clears the flags below.
        #[cfg(feature = "dshot-debug")]
        {
            let s    = pac::DMA1.st(4);
            let en   = s.cr().read().en();
            let ndtr = s.ndtr().read().ndt();
            let isr  = pac::DMA1.isr(1).read();
            dbg.frames += 1;
            // Loop cadence: gap since the previous iteration = the real inter-frame interval.
            let now = Instant::now();
            if let Some(last) = dbg.last_loop {
                let gap = (now - last).as_micros() as u32;
                if gap > dbg.max_gap_us { dbg.max_gap_us = gap; }
                if gap > 5_000 { dbg.long_gaps += 1; }   // slept past ≥2 ticks
                if gap < 1_000 { dbg.short_gaps += 1; }  // catch-up burst
            }
            dbg.last_loop = Some(now);
            if isr.tcif(0)      { dbg.tc += 1; }
            if isr.teif(0)      { dbg.teif += 1; }
            if isr.feif(0)      { dbg.feif += 1; }
            if isr.dmeif(0)     { dbg.dmeif += 1; }
            if en               { dbg.stuck_en += 1; }   // prev frame still running ≥ 2 ms = wedged
            if !en && ndtr != 0 { dbg.short_xfer += 1; } // disabled early, didn't move all 72 words
            if dbg.frames % 500 == 0 {
                info!("DSHOT dbg: frames={=u32} tc={=u32} skipped={=u32} stuck_en={=u32} short={=u32} teif={=u32} feif={=u32} dmeif={=u32} ndtr={=u16} en={=bool}",
                      dbg.frames, dbg.tc, dbg.skipped, dbg.stuck_en, dbg.short_xfer,
                      dbg.teif, dbg.feif, dbg.dmeif, ndtr, en);
                info!("DSHOT dbg: cadence max_gap={=u32}us long_gaps(>5ms)={=u32} short_gaps(<1ms)={=u32} (steady 500Hz ⇒ max≈2000, 0, 0)",
                      dbg.max_gap_us, dbg.long_gaps, dbg.short_gaps);
                dbg.max_gap_us = 0; dbg.long_gaps = 0; dbg.short_gaps = 0;
                // #4: surface the snapshot to telemetry_task → STATUSTEXT → GCS.
                debug_stats::publish(dbg.frames, dbg.tc, dbg.skipped, dbg.stuck_en,
                                     dbg.teif + dbg.feif + dbg.dmeif, dbg.last_pin_mask);
            }
        }

        let outputs  = *STATE.motor_outputs.lock().await;
        let is_armed = *STATE.armed.lock().await;

        #[cfg_attr(any(feature = "bench-force", feature = "spin-all"), allow(unused_variables))]
        let motors = if is_armed {
            // Arming voids any pending bench test so it can never carry into flight.
            *STATE.motor_test.lock().await = None;
            [
                outputs.m1.max(MOTOR_IDLE),
                outputs.m2.max(MOTOR_IDLE),
                outputs.m3.max(MOTOR_IDLE),
                outputs.m4.max(MOTOR_IDLE),
            ]
        } else {
            // Disarmed: all motors zero, unless a MAV_CMD_DO_MOTOR_TEST override is
            // active. It only runs on the ground (Idle) and self-expires by time,
            // so it cannot spin a motor in any other state.
            let mut m = [0.0f32; MOTORS];
            let mut test = STATE.motor_test.lock().await;
            if let Some(t) = *test {
                if Instant::now() >= t.until || state::get() != FlightState::Idle {
                    *test = None; // expired, or no longer on the ground → clear
                } else if (1..=MOTORS as u8).contains(&t.idx) {
                    m[(t.idx - 1) as usize] = t.throttle;
                }
            }
            m
        };

        // ⚠ BENCH ONLY (PROPS OFF): override the mixer with a constant throttle on one
        // motor so a steady DSHOT stream sits on its pin for probing — no command,
        // arming, or watchdog needed. Overrides the result above. NEVER FLY.
        #[cfg(feature = "bench-force")]
        let motors = {
            let mut m = [0.0f32; MOTORS];
            m[(BENCH_FORCE_MOTOR - 1) as usize] = BENCH_FORCE_THROTTLE;
            m
        };

        // ⚠ BENCH ONLY (PROPS OFF): CYCLE zeros → throttle → zeros forever, so the ESC
        // always gets a fresh zero-throttle arming window no matter when it powers up
        // (BLHeli_S refuses to arm into a non-zero throttle, so a latched 0.30 stream
        // is unarm-able for an ESC that boots late). Overrides the result above. NEVER FLY.
        #[cfg(feature = "spin-all")]
        let motors = {
            let phase = spin_start.elapsed().as_secs() % (SPIN_ALL_ARM_SECS + SPIN_ALL_ON_SECS);
            if phase < SPIN_ALL_ARM_SECS {
                spin_announced = false;
                [0.0f32; MOTORS] // arming window: continuous zero-throttle stream
            } else {
                if !spin_announced {
                    info!("SPIN-ALL: throttle burst — all 4 motors @ {} for {}s (⚠ PROPS OFF)",
                          SPIN_ALL_THROTTLE, SPIN_ALL_ON_SECS);
                    spin_announced = true;
                }
                [SPIN_ALL_THROTTLE; MOTORS]
            }
        };

        unsafe {
            // DMA must be idle BEFORE touching the buffer — rewriting DSHOT_BUF
            // mid-transfer corrupts the frame on the wire. Skip the tick instead.
            let busy = pac::DMA1.st(4).cr().read().en();
            let _sent = if busy {
                false
            } else {
                fill_buf(&mut *(&raw mut DSHOT_BUF), motors);
                compiler_fence(Ordering::SeqCst);
                dshot_send()
            };
            #[cfg(feature = "dshot-debug")]
            if !_sent { dbg.skipped += 1; }

            // Stay awake until the frame drains, THEN yield: the executor's WFE
            // idle-sleep clock-gates the TIM3/DMA path and strands the frame
            // mid-transfer. ~30 µs spin, bounded.
            let st = pac::DMA1.st(4);
            let mut guard = 0u32;
            while st.cr().read().en() && guard < 20_000 {
                guard += 1;
                core::hint::spin_loop();
            }
        }

        // ⚠ BENCH (`--features dshot-debug`): on the rising edge of "a motor was just
        // commanded" (test fired / spin-all engaged), verify stages 1-3 of the chain:
        // the encoded DSHOT frame (#1) and — while this very frame is on the wire — the
        // pin-toggle probe (#3). The probe runs right after dshot_send so its ~60 µs
        // window overlaps the live ~30 µs transmission instead of an idle inter-frame gap.
        #[cfg(feature = "dshot-debug")]
        {
            let active = motors.iter().any(|&t| t > 0.0);
            if active && !dbg.prev_active {
                let f: [u16; MOTORS] =
                    core::array::from_fn(|i| encode_dshot(throttle_to_dshot(motors[i])));
                let word0 = unsafe { (*core::ptr::addr_of!(DSHOT_BUF))[0] };
                info!("DSHOT dbg: COMMANDED thr=[{=f32} {=f32} {=f32} {=f32}] frames=[{=u16} {=u16} {=u16} {=u16}] buf[0]={=u16} (T1H={=u16} T0H={=u16})",
                      motors[0], motors[1], motors[2], motors[3],
                      f[0], f[1], f[2], f[3], word0, T1H, T0H);

                let mask = dshot_probe_pins(60);
                dbg.last_pin_mask = mask;
                info!("DSHOT dbg: pin-probe mask=0b{=u8:04b} order(PB4,PB5,PB0,PB1) — 1=pad physically toggled high",
                      mask);

                // #5: bidir eRPM — scaffold returns None until the capture is bench-validated.
                #[cfg(feature = "dshot-bidir")]
                if let Some(rpm) = capture_erpm_raw(0).and_then(|raw| decode_erpm(raw, MOTOR_POLES)) {
                    info!("DSHOT dbg: eRPM(M1) = {=u32}", rpm);
                }
            }
            dbg.prev_active = active;
        }
    }
}
