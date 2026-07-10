#![no_std]
#![no_main]

mod ahrs;
mod estimator;
mod health;
mod navigation;
mod pid;
mod state;
mod telemetry;
mod types;

mod sensors;    // baro, battery, flow, gps, imu, mag, rc
mod actuators;  // motor, servo
mod status;     // led

use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, gpio::{Input, Level, Output, Pull, Speed}, mode::Async, spi::{self, Spi}, time::Hertz};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker, Timer};
use state::FlightState;
use types::FlightMode;
use defmt::info;
use {defmt_rtt as _};

// ── Hardware watchdog ─────────────────────────────────────────────────────────
// IWDG1 base: 0x5800_4800. LSI ≈ 32 kHz, prescaler /256 → ~125 Hz.
// Reload 250 → timeout 2.0 s. Petted by control_task every 250 ticks (0.5 s).
// Any hang longer than 2 s triggers a hardware reset.
fn init_watchdog() {
    unsafe {
        let base = 0x5800_4800u32 as *mut u32;
        base.add(0).write_volatile(0x5555); // KR: unlock PR + RLR write access
        base.add(1).write_volatile(6);      // PR: /256 prescaler
        base.add(2).write_volatile(250);    // RLR: 250 / 125 Hz = 2.0 s
        base.add(0).write_volatile(0xAAAA); // KR: reload counter
        base.add(0).write_volatile(0xCCCC); // KR: start IWDG
    }
}

#[inline(always)]
pub fn pet_watchdog() {
    unsafe { (0x5800_4800u32 as *mut u32).write_volatile(0xAAAA); }
}

pub static STATE: types::SharedState = types::SharedState::new();

/// Remove-before-flight pin state, sampled by arming_task (which owns the pin).
/// Defaults to "installed" (arm denied) until first sampled.
pub static SAFETY_PIN_INSTALLED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(true);

/// Remove-before-flight pin gate — DISABLED 2026-07-09 for bench bring-up (no
/// jumper hardware fitted). Set true and fit the PA0→GND jumper before field
/// flights: jumper OUT (pin open/high) = clear to arm.
pub const SAFETY_PIN_ENABLED: bool = false;

/// Whether the RC gates (arm switch, throttle-low, failsafe kill) are in
/// force. Always true in normal builds. Under `rc-optional` (radio lost),
/// false until an SBUS link has been seen this boot — an absent receiver
/// cannot gate MAVLink arming or kill motors, but one that HAS been seen
/// keeps full authority, including mid-flight link-loss motor cut.
pub fn rc_gates_active() -> bool {
    #[cfg(feature = "rc-optional")]
    return sensors::rc::RC_EVER_SEEN.load(core::sync::atomic::Ordering::Relaxed);
    #[cfg(not(feature = "rc-optional"))]
    true
}

/// Pre-arm gates. Every arm path (RC in arming_task, MAVLink cmd 400 in
/// telemetry) MUST go through this single function.
pub async fn pre_arm_check(mode: FlightMode) -> Result<(), &'static str> {
    use core::sync::atomic::Ordering;

    if SAFETY_PIN_ENABLED && SAFETY_PIN_INSTALLED.load(Ordering::Relaxed) {
        return Err("remove-before-flight pin installed");
    }

    let health = *STATE.sensor_health.lock().await;
    let mode_needs_gps = matches!(mode,
        FlightMode::PositionHold | FlightMode::Auto
        | FlightMode::ReturnToHome | FlightMode::FollowMe);
    // Every mode except Stabilise runs an altitude PID off the baro.
    let mode_needs_baro = !matches!(mode, FlightMode::Stabilise);

    if !health.imu_ok {
        return Err("IMU not healthy");
    }
    if mode_needs_baro && !health.baro_ok {
        return Err("mode requires a live barometer");
    }
    if mode_needs_gps && !health.gps_ok {
        return Err("mode requires nav-ready GPS (3D fix + accuracy)");
    }
    if mode_needs_gps && !health.mag_ok {
        return Err("mode requires a calibrated compass");
    }
    // Also catches an uncalibrated V_DIVIDER (reads 0 V = critical).
    if STATE.battery.lock().await.critical {
        return Err("battery critical (check pack / V_DIVIDER calibration)");
    }
    Ok(())
}

/// Shared SPI1 bus — IMU (CS=PA4) and baro (CS=PA8) are both on this bus.
/// Initialised in main() before any task is spawned.
pub type SpiBus = Spi<'static, Async, spi::mode::Master>;
pub static SPI1_BUS: Mutex<CriticalSectionRawMutex, Option<SpiBus>> = Mutex::new(None);

bind_interrupts!(pub struct Irqs {
    USART1       => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
    USART2       => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
    USART3       => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART3>;
    UART4        => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::UART4>;
    //SPI1       => embassy_stm32::spi::InterruptHandler<embassy_stm32::peripherals::SPI1>;
    DMA1_STREAM1 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH1>;
    DMA1_STREAM2 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH2>;
    DMA1_STREAM3 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH3>;
    DMA1_STREAM5 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH5>;
    DMA2_STREAM0 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA2_CH0>;
    DMA2_STREAM3 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA2_CH3>;
    DMA2_STREAM5 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA2_CH5>;
    DMA2_STREAM6 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA2_CH6>;
});

/// 500 Hz control loop: AHRS fusion → PID cascade → motor mix → STATE.motor_outputs.
#[embassy_executor::task]
async fn control_task() {
    use ahrs::MadgwickFilter;
    use pid::{mix_quad_x, FlightPids, LowPass3};

    // Gyro low-pass cutoff for the rate loop (attenuates motor noise on the D-term).
    const GYRO_LPF_HZ: f32 = 90.0;
    // Crash/tumble cutoff: disarm if |roll| or |pitch| exceeds this for CRASH_TICKS.
    const CRASH_ANGLE_RAD: f32 = 1.31;   // ~75°
    const CRASH_TICKS: u32 = 100;        // 0.2 s sustained @ 500 Hz

    let mut filter = MadgwickFilter::new(0.1, 1.0 / 500.0);
    let mut gyro_lpf = LowPass3::new(GYRO_LPF_HZ, 1.0 / 500.0);
    let mut pids = FlightPids::default();
    let mut ticker = Ticker::every(Duration::from_hz(500));
    let mut wdg_tick: u32 = 0;
    let mut crash_ticks: u32 = 0;

    loop {
        ticker.next().await;
        wdg_tick += 1;
        if wdg_tick % 250 == 0 {   // every 0.5 s at 500 Hz — well within 2 s timeout
            crate::pet_watchdog();
        }

        let imu      = *STATE.imu_data.lock().await;
        let mag      = *STATE.mag_data.lock().await;
        let is_armed = *STATE.armed.lock().await;
        let nav_cmd  = *STATE.nav_command.lock().await;
        let rc       = *STATE.rc_input.lock().await;

        // Rate loop uses filtered gyro; Madgwick uses raw to avoid added lag.
        let gyro_f = gyro_lpf.apply(imu.gyro);

        let setpoint = if nav_cmd.autonomous {
            nav_cmd.attitude_setpoint
        } else {
            rc.to_attitude_setpoint()
        };

        // usable() = calibrated AND fresh — never fuse a stale compass snapshot.
        let quat = if mag.usable() {
            filter.update_with_mag(
                imu.gyro, imu.accel,
                types::Vec3 { x: mag.x, y: mag.y, z: mag.z },
            )
        } else {
            filter.update(imu.gyro, imu.accel)
        };
        *STATE.attitude.lock().await = quat;
        let euler = filter.euler();

        // Bench IMU bring-up: ~2 Hz attitude readout — tilt the board and watch
        // roll/pitch/yaw track to confirm IMU axes + fusion. Bench build only.
        // yawNED is the NED heading (ahrs::ned_yaw): 0 = North, CW-positive —
        // the bench acceptance test "yaw increases rotating clockwise from
        // above" checks THIS value, not the raw (CCW-positive) Madgwick yaw.
        #[cfg(feature = "nucleo-vcp")]
        if wdg_tick % 250 == 0 {
            const R2D: f32 = 180.0 / core::f32::consts::PI;
            // velD: estimator NED down-velocity (m/s) — the lift acceptance
            // test wants this clearly NEGATIVE while raising the board.
            let est = *STATE.pos_estimate.lock().await;
            info!("ATT  roll={=f32}deg  pitch={=f32}deg  yawNED={=f32}deg  velD={=f32}m/s",
                  euler.roll * R2D, euler.pitch * R2D, ahrs::ned_yaw(&quat) * R2D, est.vel_d);
        }

        // Crash/tumble cutoff: sustained extreme tilt while armed → kill motors, latch Fault.
        if is_armed && (euler.roll.abs() > CRASH_ANGLE_RAD || euler.pitch.abs() > CRASH_ANGLE_RAD) {
            crash_ticks += 1;
            if crash_ticks > CRASH_TICKS {
                *STATE.armed.lock().await = false;
                state::set(FlightState::Fault);
                *STATE.motor_outputs.lock().await = Default::default();
                pids.reset_all();
                defmt::warn!("Crash/tumble: extreme tilt — disarmed + Fault");
                crash_ticks = 0;
                continue;
            }
        } else {
            crash_ticks = 0;
        }

        // RC failsafe kills motors only while the RC gates are live (always in
        // normal builds; under `rc-optional`, only once SBUS has been seen).
        if !is_armed || (rc.failsafe && rc_gates_active()) {
            pids.reset_all();
            *STATE.motor_outputs.lock().await = Default::default();
            continue;
        }

        let (roll_out, pitch_out, yaw_out) = pids.update(&setpoint, &euler, gyro_f);
        let outputs = mix_quad_x(setpoint.throttle, roll_out, pitch_out, yaw_out);
        *STATE.motor_outputs.lock().await = outputs;

        // PID-tuning stream (~50 Hz), capture via RTT: `cargo run --features tune-log`.
        #[cfg(feature = "tune-log")]
        if wdg_tick % 10 == 0 {
            info!("TUNE spR={=f32} spP={=f32} mR={=f32} mP={=f32} gx={=f32} gy={=f32} gz={=f32} oR={=f32} oP={=f32} oY={=f32}",
                  setpoint.roll, setpoint.pitch, euler.roll, euler.pitch,
                  gyro_f.x, gyro_f.y, gyro_f.z, roll_out, pitch_out, yaw_out);
        }
    }
}

/// 50 Hz: arm/disarm via RC switch (Ch5 > mid), throttle-low interlock.
///
/// Arm requires: switch HIGH + throttle < 5 % + no failsafe + not Fault +
/// pre_arm_check(). Disarm (switch LOW or failsafe) is honoured in EVERY
/// state including Fault — the pilot's switch is an absolute kill.
///
/// `safety_pin`: remove-before-flight jumper (PA0→GND, pull-up: inserted =
/// LOW = arm denied). Gates only the disarmed→armed transition — never
/// checked in flight, so a flaky wire can't force a mid-air disarm.
#[embassy_executor::task]
async fn arming_task(safety_pin: Input<'static>) {
    use core::sync::atomic::Ordering;
    use embassy_time::Instant;

    let mut ticker = Ticker::every(Duration::from_hz(50));
    // Deny warns rate-limited: the deny path holds every tick while the switch is up.
    let mut last_deny_log: Option<Instant> = None;

    #[cfg(feature = "rc-optional")]
    defmt::warn!("⚠ rc-optional build: until an SBUS link is seen, MAVLink arming \
                  skips the RC gates and GCS disarm is the ONLY kill switch");

    loop {
        ticker.next().await;

        SAFETY_PIN_INSTALLED.store(safety_pin.is_low(), Ordering::Relaxed);

        let rc       = *STATE.rc_input.lock().await;
        let is_armed = *STATE.armed.lock().await;
        let rc_gates = rc_gates_active();

        if is_armed {
            if rc_gates && (!rc.arm || rc.failsafe) {
                *STATE.armed.lock().await = false;
                // A latched Fault stays latched; recovery is owned by the fault source.
                if state::get() != FlightState::Fault {
                    state::set(FlightState::Idle);
                }
                info!("Disarmed");
            } else if state::get() == FlightState::Armed {
                // Promote Armed → Flying once real power is applied — pilot stick
                // OR autonomous throttle. Without the autonomous term an Auto
                // takeoff flies in state Armed, and the MAVLink disarm handler's
                // in-air guard (Flying|Landing) would not protect it.
                let nav = *STATE.nav_command.lock().await;
                if rc.throttle > 0.15
                    || (nav.autonomous && nav.attitude_setpoint.throttle > 0.15)
                {
                    state::set(FlightState::Flying);
                }
            }
        } else if state::get() == FlightState::Fault {
            // Never arm out of Fault.
            continue;
        } else if rc.arm && rc.throttle < 0.05 && !rc.failsafe {
            let mode = STATE.effective_mode().await;
            match pre_arm_check(mode).await {
                Ok(()) => {
                    *STATE.armed.lock().await = true;
                    state::set(FlightState::Armed);
                    info!("Armed");
                    last_deny_log = None;
                }
                Err(reason) => {
                    let due = last_deny_log.map_or(true, |t| t.elapsed() > Duration::from_secs(1));
                    if due {
                        defmt::warn!("Arm denied: {=str}", reason);
                        last_deny_log = Some(Instant::now());
                    }
                }
            }
        } else {
            last_deny_log = None;
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    
    use embassy_stm32::rcc::*;

    // SupplyConfig::Default writes SDEN=1 LDOEN=1, which is identical to the
    // H723 POR reset value — making it a no-op and keeping ACTVOSRDY asserted.
    // Any other SupplyConfig variant writes a different CR3 value, drops
    // ACTVOSRDY, and hangs Embassy's init loop. CR3 is write-once per POR.
    config.rcc.supply_config = SupplyConfig::Default;
    config.rcc.voltage_scale = VoltageScale::Scale1;

    config.rcc.hsi = Some(HSIPrescaler::Div1);
    config.rcc.csi = true;

    config.rcc.pll1 = Some(Pll {
        source: PllSource::Hsi,
        prediv: PllPreDiv::Div4,    // 64 MHz / 4  = 16 MHz
        mul:    PllMul::Mul50,      // 16 MHz × 50 = 800 MHz VCO
        divp:   Some(PllDiv::Div2), // 800 / 2 = 400 MHz → SYSCLK
        divq:   Some(PllDiv::Div8), // 800 / 8 = 100 MHz → SPI/FDCAN
        divr:   None,
    });

    config.rcc.sys      = Sysclk::Pll1P;         // SYSCLK = 400 MHz
    config.rcc.ahb_pre  = AHBPrescaler::Div2;    // HCLK   = 200 MHz
    config.rcc.apb1_pre = APBPrescaler::Div2;    // APB1   = 100 MHz → TIM3 timer = 200 MHz
    config.rcc.apb2_pre = APBPrescaler::Div2;    // APB2   = 100 MHz
    config.rcc.apb3_pre = APBPrescaler::Div2;    // APB3   = 100 MHz
    config.rcc.apb4_pre = APBPrescaler::Div2;    // APB4   = 100 MHz

    // ADC kernel clock from per_ck (HSI) — without it ADC conversions never complete.
    // ⚠ Calibrate V_DIVIDER (battery.rs) at bring-up before trusting the low-battery
    // failsafe: a wrong reading reads "critical" and forces RTH/Land.
    config.rcc.mux.adcsel = mux::Adcsel::Per;

    // IWDG runs off the LSI, independent of RCC — start it BEFORE embassy init
    // so a hang inside init (e.g. wrong SupplyConfig) resets instead of bricking.
    // Petted ONLY by control_task; individual sensor-task deaths are covered by
    // health_task freshness flags, not the watchdog.
    init_watchdog();

    let p = embassy_stm32::init(config);

    // Initialise shared SPI1 bus before spawning — 12.5 MHz, Mode 0 works for
    // both ICM-42688-P (max 24 MHz) and MS5611 (max 20 MHz).
    {
        let mut spi_cfg = spi::Config::default();
        spi_cfg.frequency = Hertz(12_500_000);
        spi_cfg.mode      = spi::MODE_0;
        // MOSI: PA7 on the custom FC; PD7 on the Nucleo, where PA7 is hard-wired
        // to the on-board Ethernet PHY (RMII_CRS_DV) and unusable for SPI.
        #[cfg(not(feature = "nucleo"))]
        let spi = Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH0, Irqs, spi_cfg);
        #[cfg(feature = "nucleo")]
        let spi = Spi::new(p.SPI1, p.PA5, p.PD7, p.PA6, p.DMA2_CH3, p.DMA2_CH0, Irqs, spi_cfg);
        *SPI1_BUS.lock().await = Some(spi);
    }

    // Status LED: PG7 on the custom FC (active-LOW); LD2/PE1 on the Nucleo
    // (active-HIGH). Both start OFF. led_task handles the polarity.
    #[cfg(not(feature = "nucleo"))]
    let led = Output::new(p.PG7, Level::High, Speed::Low);
    #[cfg(feature = "nucleo")]
    let led = Output::new(p.PE1, Level::Low, Speed::Low);

    // Remove-before-flight pin: PA0→GND jumper, internal pull-up.
    // Removed (open) = HIGH = clear to arm; installed (shorted) = LOW = denied.
    let safety_pin = Input::new(p.PA0, Pull::Up);

    // Gripper-jaw microswitch (PC2 → GND, closed = weed held = LOW). Read by
    // navigation_task's grip check under `--features grip-sense`.
    let grip_pin = Input::new(p.PC2, Pull::Up);

    spawner.spawn(status::led::led_task(led).unwrap());
    spawner.spawn(sensors::imu::imu_task(p.PA4).unwrap());
    #[cfg(not(feature = "pin-test"))]
    spawner.spawn(actuators::motor::motor_task(p.TIM3, p.PB4, p.PB5, p.PB0, p.PB1).unwrap());
    // pin-test: motor pins as plain GPIO for DMM continuity checks — nothing spins.
    #[cfg(feature = "pin-test")]
    spawner.spawn(actuators::motor::pin_test_task(p.PB4, p.PB5, p.PB0, p.PB1).unwrap());
    spawner.spawn(sensors::rc::rc_task(p.USART2, p.PA2, p.PA3, p.DMA1_CH5, Irqs).unwrap());
    // MAVLink on USART3. Default = Pi header pins (PB11/PB10); `nucleo-vcp`
    // routes it to the ST-Link VCP (PD9/PD8) for prop-off bench testing over USB.
    #[cfg(not(feature = "nucleo-vcp"))]
    spawner.spawn(telemetry::telemetry_task(p.USART3, p.PB11, p.PB10, p.DMA1_CH3, p.DMA1_CH1, Irqs).unwrap());
    #[cfg(feature = "nucleo-vcp")]
    spawner.spawn(telemetry::telemetry_task(p.USART3, p.PD9, p.PD8, p.DMA1_CH3, p.DMA1_CH1, Irqs).unwrap());
    spawner.spawn(sensors::baro::baro_task(p.PA8).unwrap());
    spawner.spawn(sensors::gps::gps_task(p.USART1, p.PA10, p.PA9, p.DMA2_CH6, p.DMA2_CH5, Irqs).unwrap());
    // Battery monitor (ADC3: PC0 = pack divider, PF3 = ESC CUR pad).
    // Bench with no pack reads ~0 V → "critical" (arming blocked).
    spawner.spawn(sensors::battery::battery_task(p.ADC3, p.PC0, p.PF3).unwrap());
    spawner.spawn(navigation::navigation_task(grip_pin).unwrap());
    spawner.spawn(estimator::estimator_task().unwrap());
    spawner.spawn(health::health_task().unwrap());
    spawner.spawn(control_task().unwrap());
    spawner.spawn(arming_task(safety_pin).unwrap());
    // Flow (MTF-02P): safe unwired — flow.valid stays false, consumers fall back to GPS/baro.
    spawner.spawn(sensors::flow::flow_task(p.UART4, p.PC11, p.DMA1_CH2, Irqs).unwrap());
    spawner.spawn(actuators::payloads::servo::servo_task(p.TIM4, p.PD12, p.PD13, p.PD14, p.PD15).unwrap());
    spawner.spawn(sensors::mag::mag_task(p.I2C1, p.PB8, p.PB9).unwrap());

    state::set(FlightState::Idle);

    loop {
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    unsafe {
        // Explicit zero-throttle DSHOT frame so ESCs stop without waiting for
        // signal-loss timeout (drains/aborts any in-flight DMA first).
        actuators::motor::emergency_stop();
        // Turn off the status LED via BSRR (+0x18, offset(6)) — atomic set/reset,
        // can't clobber other pins on the port. H7 GPIO bases are 0x5802_xxxx
        // (AHB4) — never F4-era 0x4002_xxxx. Update if the LED pin moves.
        #[cfg(not(feature = "nucleo"))] // custom FC: PG7 active-low → set HIGH = off
        let (gpio_base, bsrr) = (0x5802_1800u32, 1u32 << 7);
        #[cfg(feature = "nucleo")]      // Nucleo: PE1 active-high → reset LOW = off
        let (gpio_base, bsrr) = (0x5802_1000u32, 1u32 << (1 + 16));
        (gpio_base as *mut u32).offset(6).write_volatile(bsrr);
    }
    loop {}
}