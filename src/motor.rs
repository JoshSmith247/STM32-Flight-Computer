//! Motor output — DSHOT600 via TIM3 burst DMA.
//!
//! TIM3 runs at 200 MHz (APB1=100 MHz × 2), ARR=332 → 333-tick period = 1.667 µs = 600 kHz.
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
//!
//! !! VERIFY before first run: open RM0468 Table 105, search for "TIM3_UP", confirm
//! DMAMUX_TIM3_UP matches that row's request ID. Wrong value = DMA never fires =
//! motors silently disabled (safe to diagnose, not dangerous).

use core::sync::atomic::{compiler_fence, Ordering};

use defmt::info;
use embassy_stm32::{pac, peripherals, Peri};
use embassy_time::{Duration, Ticker, Timer};

use crate::STATE;

// ── DSHOT600 constants ───────────────────────────────────────────────────────

const ARR:  u16 = 332;  // TIM period = 333 ticks at 200 MHz → 1.667 µs = 600 kHz
const T1H:  u16 = 250;  // 75%   duty — logic 1
const T0H:  u16 = 125;  // 37.5% duty — logic 0

const SLOTS:   usize = 18;             // 16 data bits + 2 reset
const MOTORS:  usize = 4;
const BUF_LEN: usize = SLOTS * MOTORS; // 72 u16 words

// TIM3 base = 0x4000_4000; DMAR at +0x4C
const TIM3_DMAR: u32 = 0x4000_404C;

// DMAMUX1 request ID for TIM3_UP on STM32H723.
// Verify against RM0468 Table 105 before first run.
const DMAMUX_TIM3_UP: u8 = 22;

// ── DSHOT frame encoding ─────────────────────────────────────────────────────

fn throttle_to_dshot(t: f32) -> u16 {
    if t <= 0.0 { 0 } else { (48.0 + t * (2047.0 - 48.0)) as u16 }
}

fn encode_dshot(thr: u16) -> u16 {
    let payload = thr << 1; // telemetry request = 0
    let crc = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0F;
    (payload << 4) | crc
}

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

// ── DMA buffer ───────────────────────────────────────────────────────────────

// AXISRAM (0x2400_0000) is DMA-accessible on STM32H723; the linker places
// all statics there, so no special link section is needed.
static mut DSHOT_BUF: [u16; BUF_LEN] = [0u16; BUF_LEN];

// ── Hardware init ────────────────────────────────────────────────────────────

unsafe fn dshot_init() {
    use pac::gpio::vals::{Moder, Ospeedr, Ot};
    use pac::timer::vals::OcmGp;
    use pac::dma::vals::{Dir, Pl, Size};

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

    // Configure TIM3 for DSHOT600 bit rate
    let tim = pac::TIM3;
    tim.psc().write(|w| *w = 0u16);              // prescaler = 0 → 200 MHz
    tim.arr().write(|w| w.set_arr(ARR));          // period = 333 ticks = 1.667 µs

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
    while s.cr().read().en() {}

    // Clear all stream-4 flags via the high-register IFCR (streams 4-7 are in ifcr(1))
    // Stream 4 is at local index 0 within that register.
    pac::DMA1.ifcr(1).write(|w| {
        w.set_tcif(0, true); w.set_htif(0, true);
        w.set_teif(0, true); w.set_dmeif(0, true);
        w.set_feif(0, true);
    });

    s.par().write(|w| *w = TIM3_DMAR); // fixed peripheral address

    s.cr().write(|w| {
        w.set_dir(Dir::MemoryToPeripheral);
        w.set_minc(true);              // advance through DSHOT_BUF
        w.set_pinc(false);             // DMAR address is fixed
        w.set_msize(Size::Bits16);
        w.set_psize(Size::Bits16);
        w.set_pl(Pl::VeryHigh);
        w.set_circ(false);             // one-shot per frame
        w.set_en(false);
    });
}

unsafe fn dshot_send() {
    let s = pac::DMA1.st(4);

    // Frame takes 18 × 1.667 µs ≈ 30 µs; at 500 Hz ticks (2 ms apart) the
    // previous transfer is always done.  Spin just in case of a missed tick.
    while s.cr().read().en() { core::hint::spin_loop(); }

    pac::DMA1.ifcr(1).write(|w| {
        w.set_tcif(0, true); w.set_htif(0, true);
        w.set_teif(0, true); w.set_dmeif(0, true);
        w.set_feif(0, true);
    });

    s.m0ar().write(|w| *w = core::ptr::addr_of!(DSHOT_BUF) as u32);
    s.ndtr().write(|w| w.set_ndt(BUF_LEN as u16));
    s.cr().modify(|w| w.set_en(true)); // timer update event fires DMA
}

// ── Task ─────────────────────────────────────────────────────────────────────

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
    Timer::after(Duration::from_millis(2000)).await;

    info!("Motors: DSHOT600 running (TIM3 + DMA1_CH4)");

    let mut ticker = Ticker::every(Duration::from_hz(500));

    loop {
        ticker.next().await;

        let outputs  = *STATE.motor_outputs.lock().await;
        let is_armed = *STATE.armed.lock().await;

        let motors = if is_armed {
            [outputs.m1, outputs.m2, outputs.m3, outputs.m4]
        } else {
            [0.0f32; MOTORS]
        };

        unsafe {
            fill_buf(&mut *(&raw mut DSHOT_BUF), motors);
            compiler_fence(Ordering::SeqCst);
            dshot_send();
        }
    }
}
