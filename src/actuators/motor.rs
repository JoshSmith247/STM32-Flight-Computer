//! Motor output - DSHOT300 (the fitted HGLRC BLHeli_S ESC only parses 300)
//! via TIM3 burst DMA on DMA1_CH4 (reserved - don't allocate elsewhere).
//! Pinout (TIM3 AF2): M1->PB4/CH1, M2->PB5/CH2, M3->PB0/CH3, M4->PB1/CH4.

use core::sync::atomic::{compiler_fence, Ordering};

use defmt::info;
use embassy_stm32::{pac, peripherals, Peri};
use embassy_time::{Duration, Instant, Ticker};

use crate::state::{self, FlightState};
use crate::STATE;

// DSHOT300 timing constants

const ARR:  u16 = 332;  // TIM period = 333 ticks; at 100 MHz (PSC=1) -> 3.33 us/bit = DSHOT300
const T1H:  u16 = 250;  // 75% duty - logic 1
const T0H:  u16 = 125;  // 37.5% duty - logic 0

const SLOTS:   usize = 18;             // 16 data bits + 2 reset
const MOTORS:  usize = 4;
const BUF_LEN: usize = SLOTS * MOTORS; // 72 u16 halfwords (DMA to 16-bit TIM3_DMAR)

// Idle floor while ARMED (a stopped rotor has no attitude authority).
// WARNING: Motors spin at this the moment the craft arms - props clear!
const MOTOR_IDLE: f32 = 0.04;

// DMAMUX1 request ID for TIM3_UP (RM0468 Table 105) - verified on hardware 2026-06-20.
// Wrong value = DMA never fires = motors silently dead.
const DMAMUX_TIM3_UP: u8 = 27;

// DSHOT frame encoding

fn throttle_to_dshot(t: f32) -> u16 {
    let t = t.clamp(0.0, 1.0);
    if t <= 0.0 { 0 } else { (48.0 + t * (2047.0 - 48.0)) as u16 }
}

fn encode_dshot(thr: u16) -> u16 {
    let payload = thr << 1; // telemetry request bit = 0
    let csum = payload ^ (payload >> 4) ^ (payload >> 8);
    let crc = csum & 0x0F;
    (payload << 4) | crc
}

// Physical motor-order remap: MOTOR_SLOT[i] = TIM3 slot driving mixer motor i+1
// (bench --sequence, 2026-07-09; re-verified on the new ESC+board 2026-09-20).
const MOTOR_SLOT: [usize; MOTORS] = [1, 3, 0, 2];

// Buffer is u16: the DMA does halfword transfers to TIM3's 16-bit DMAR/CCR registers.
fn fill_buf(buf: &mut [u16; BUF_LEN], motors: [f32; MOTORS]) {
    let frames: [u16; MOTORS] =
        core::array::from_fn(|i| encode_dshot(throttle_to_dshot(motors[i])));

    for bit in 0..16 {
        for (m, &frame) in frames.iter().enumerate() {
            buf[bit * MOTORS + MOTOR_SLOT[m]] = if (frame >> (15 - bit)) & 1 == 1 { T1H } else { T0H };
        }
    }
    for slot in 16..SLOTS {
        for m in 0..MOTORS { buf[slot * MOTORS + m] = 0; }
    }
}

// DMA buffer

// AXISRAM (0x2400_0000) is DMA-accessible on STM32H723; the linker places
// all statics there, so no special link section is needed.
static mut DSHOT_BUF: [u16; BUF_LEN] = [0u16; BUF_LEN];

// Hardware init

unsafe fn dshot_init() {
    use pac::gpio::vals::{Moder, Ospeedr, Ot};
    use pac::timer::vals::OcmGp;
    use pac::dma::vals::{Burst, Dir, Dmdis, Pl, Size};

    // Enable peripheral clocks
    pac::RCC.ahb4enr().modify(|w| w.set_gpioben(true));
    pac::RCC.apb1lenr().modify(|w| w.set_tim3en(true));
    pac::RCC.ahb1enr().modify(|w| w.set_dma1en(true));
    cortex_m::asm::dsb();

    // Configure PB0, PB1, PB4, PB5 -> TIM3 AF2
    // All pins <= 7 so AFRL = afr(0)
    let gp = pac::GPIOB;
    for &pin in &[0usize, 1, 4, 5] {
        gp.moder().modify(|w| w.set_moder(pin, Moder::Alternate));
        gp.ospeedr().modify(|w| w.set_ospeedr(pin, Ospeedr::VeryHighSpeed));
        gp.otyper().modify(|w| w.set_ot(pin, Ot::PushPull));
        gp.afr(0).modify(|w| w.set_afr(pin, 2)); // AF2 = TIM3
    }

    let tim = pac::TIM3;
    tim.psc().write(|w| *w = 1u16);              // prescaler = 1 -> 100 MHz -> 300 kbit/s
    tim.arr().write(|w| w.set_arr(ARR));          // period = 333 ticks

    // PWM mode 1 with output preload; ccmr_output(0)=CH1/CH2, ccmr_output(1)=CH3/CH4.
    tim.ccmr_output(0).write(|w| {
        w.set_ocm(0, OcmGp::PwmMode1); w.set_ocpe(0, true);
        w.set_ocm(1, OcmGp::PwmMode1); w.set_ocpe(1, true);
    });
    tim.ccmr_output(1).write(|w| {
        w.set_ocm(0, OcmGp::PwmMode1); w.set_ocpe(0, true);
        w.set_ocm(1, OcmGp::PwmMode1); w.set_ocpe(1, true);
    });

    // Enable CC outputs; CCR=0 -> line LOW while DMA is idle
    tim.ccer().write(|w| {
        w.set_cce(0, true); w.set_cce(1, true);
        w.set_cce(2, true); w.set_cce(3, true);
    });
    for ch in 0..4usize { tim.ccr(ch).write(|w| w.set_ccr(0)); }

    // Burst DMA: DBA=13 (CCR1 is at TIM base+0x34, offset 13 registers from CR1),
    // DBL=3 (burst length = 4 registers: CCR1->CCR4).
    tim.dcr().write(|w| { w.set_dba(13); w.set_dbl(3); });

    // Enable update -> DMA request
    tim.dier().modify(|w| w.set_ude(true));

    // Start timer; generate update event to load preload registers into active
    tim.cr1().modify(|w| { w.set_arpe(true); w.set_cen(true); });
    tim.egr().write(|w| w.set_ug(true));

    // Route DMAMUX1 channel 4 to TIM3_UP
    pac::DMAMUX1.ccr(4).write(|w| w.set_dmareq_id(DMAMUX_TIM3_UP));

    // Configure DMA1 stream 4
    let s = pac::DMA1.st(4);
    s.cr().modify(|w| w.set_en(false));
    // Bounded: a stuck DMA must not wedge the single-threaded executor.
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

    // TIM_DMAR constraints: NO burst (DCR.DBL does the fan-out), HALFWORD transfers
    // (DMAR is 16-bit - 32-bit writes corrupt the fan-out), DIRECT mode (no FIFO).
    s.fcr().write(|w| {
        w.set_dmdis(Dmdis::Enabled); // direct mode - no FIFO
    });

    s.cr().write(|w| {
        w.set_dir(Dir::MemoryToPeripheral);
        w.set_minc(true);              // advance through DSHOT_BUF
        w.set_pinc(false);             // DMAR address is fixed
        w.set_msize(Size::Bits16);     // halfword: TIM3 DMAR/CCR are 16-bit
        w.set_psize(Size::Bits16);
        w.set_mburst(Burst::Single);   // NO burst - timer's DBL fan-out re-requests per CCR
        w.set_pburst(Burst::Single);
        w.set_pl(Pl::VeryHigh);
        w.set_circ(false);             // one-shot per frame
        w.set_en(false);
    });
}

/// Queue one DSHOT frame via DMA. Returns `false` (frame skipped) if the previous
/// DMA is still enabled - never busy-wait here or the watchdog pet starves.
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

// Emergency stop (called from panic handler)

/// Zero all motors and fire one DSHOT stop frame. Panic-handler safe: drains
/// any in-flight DMA (bounded), force-aborting a wedged stream, then sends.
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

// Task

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

    // ESC arming window: 2 s of continuous zero frames at 500 Hz - BLHeli_S
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

    info!("Motors: DSHOT300 running (TIM3 + DMA1_CH4, PSC=1)");

    let mut ticker = Ticker::every(Duration::from_hz(500));

    loop {
        ticker.next().await;

        let outputs  = *STATE.motor_outputs.lock().await;
        let is_armed = *STATE.armed.lock().await;

        let motors = if is_armed {
            // Arming voids any pending bench test so it can never carry into flight.
            *STATE.motor_test.lock().await = None;
            [
                outputs.m1.max(MOTOR_IDLE), // Return max of the value and idle, so floor idle
                outputs.m2.max(MOTOR_IDLE),
                outputs.m3.max(MOTOR_IDLE),
                outputs.m4.max(MOTOR_IDLE),
            ]
        } else {
            // Disarmed: all zero unless a MAV_CMD_DO_MOTOR_TEST override is active
            // (Idle-state only, self-expiring).
            let mut m = [0.0f32; MOTORS];
            let mut test = STATE.motor_test.lock().await;
            if let Some(t) = *test {
                if Instant::now() >= t.until || state::get() != FlightState::Idle {
                    *test = None; // expired, or no longer on the ground -> clear
                } else if (1..=MOTORS as u8).contains(&t.idx) {
                    m[(t.idx - 1) as usize] = t.throttle;
                }
            }
            m
        };

        unsafe {
            // DMA must be idle BEFORE touching the buffer - rewriting DSHOT_BUF
            // mid-transfer corrupts the frame on the wire. Skip the tick instead.
            if !pac::DMA1.st(4).cr().read().en() {
                fill_buf(&mut *(&raw mut DSHOT_BUF), motors);
                compiler_fence(Ordering::SeqCst);
                dshot_send();
            }

            // Stay awake until the frame drains, THEN yield: the executor's WFE
            // idle-sleep clock-gates TIM3/DMA and strands the frame mid-transfer.
            let st = pac::DMA1.st(4);
            let mut guard = 0u32;
            while st.cr().read().en() && guard < 20_000 {
                guard += 1;
                core::hint::spin_loop();
            }
        }

    }
}
