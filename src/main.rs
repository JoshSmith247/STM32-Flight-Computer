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

        // Filtered gyro for the rate loop (run every tick to keep it warm); the
        // Madgwick estimator below intentionally uses the raw gyro to avoid added lag.
        let gyro_f = gyro_lpf.apply(imu.gyro);

        let setpoint = if nav_cmd.autonomous {
            nav_cmd.attitude_setpoint
        } else {
            rc.to_attitude_setpoint()
        };

        let quat = if mag.valid {
            filter.update_with_mag(
                imu.gyro, imu.accel,
                types::Vec3 { x: mag.x, y: mag.y, z: mag.z },
            )
        } else {
            filter.update(imu.gyro, imu.accel)
        };
        *STATE.attitude.lock().await = quat;
        let euler = filter.euler();

        // Bench IMU bring-up: ~2 Hz attitude readout (degrees) so you can tilt the
        // board and watch roll/pitch/yaw track — confirms IMU axes + Madgwick fusion.
        // Bench build only; keeps flight logs clean. Runs while disarmed too.
        #[cfg(feature = "nucleo-vcp")]
        if wdg_tick % 250 == 0 {
            const R2D: f32 = 180.0 / core::f32::consts::PI;
            info!("ATT  roll={=f32}deg  pitch={=f32}deg  yaw={=f32}deg",
                  euler.roll * R2D, euler.pitch * R2D, euler.yaw * R2D);
        }

        // Crash/tumble cutoff: a sustained extreme tilt while armed means we've flipped
        // or hit the ground — kill motors and latch Fault so they can't keep driving in.
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

        if !is_armed || rc.failsafe {
            pids.reset_all();
            *STATE.motor_outputs.lock().await = Default::default();
            continue;
        }

        let (roll_out, pitch_out, yaw_out) = pids.update(&setpoint, &euler, gyro_f);
        let outputs = mix_quad_x(setpoint.throttle, roll_out, pitch_out, yaw_out);
        *STATE.motor_outputs.lock().await = outputs;

        // ── Blackbox / PID-tuning stream — capture via RTT (`cargo run --features tune-log`) ──
        // ~50 Hz: per-axis setpoint vs measured attitude, filtered gyro, and PID outputs, so
        // step responses and oscillation are visible for tuning. Off by default (zero cost).
        #[cfg(feature = "tune-log")]
        if wdg_tick % 10 == 0 {
            info!("TUNE spR={=f32} spP={=f32} mR={=f32} mP={=f32} gx={=f32} gy={=f32} gz={=f32} oR={=f32} oP={=f32} oY={=f32}",
                  setpoint.roll, setpoint.pitch, euler.roll, euler.pitch,
                  gyro_f.x, gyro_f.y, gyro_f.z, roll_out, pitch_out, yaw_out);
        }
    }
}

/// 50 Hz: arm/disarm the drone via RC switch, with throttle-low safety interlock.
///
/// Arm conditions (all must hold simultaneously):
///   • Arm switch HIGH  (RC Ch5 > mid)
///   • Throttle at zero (< 5 %)
///   • RC link healthy  (no failsafe)
///   • Not in Fault state
///
/// Disarm on: arm switch LOW, or RC failsafe (signal loss).
/// The control_task also zeros motors immediately on failsafe; this task
/// confirms the disarm within one 20 ms tick.
///
/// `safety_pin`: a physical "remove before flight" interlock (2-pin header on PA0/GND,
/// bridged by a pull-pin/jumper). Wired with the internal pull-up, so pin INSERTED
/// (shorts PA0 to GND) reads LOW = blocked, pin REMOVED (open) reads HIGH = clear.
/// ⚠ This only gates the disarmed→armed transition, not an already-flying aircraft —
/// checking it on every tick while armed would let a flaky wire force a mid-air disarm,
/// which is worse than the hazard it's meant to prevent. It is also fail-*open*, not
/// fail-safe: a severed wire reads identically to "pin removed" (HIGH = clear). It is a
/// discipline/reminder layer stacked on top of the RC arm switch + throttle-zero +
/// health gates below, not a certified standalone interlock — treat pulling it as part
/// of the pre-flight checklist, not a substitute for prop-off / area-clear checks.
#[embassy_executor::task]
async fn arming_task(safety_pin: Input<'static>) {
    let mut ticker = Ticker::every(Duration::from_hz(50));

    loop {
        ticker.next().await;

        if state::get() == FlightState::Fault {
            continue;
        }

        let rc       = *STATE.rc_input.lock().await;
        let is_armed = *STATE.armed.lock().await;

        if is_armed {
            if !rc.arm || rc.failsafe {
                *STATE.armed.lock().await = false;
                state::set(FlightState::Idle);
                info!("Disarmed");
            } else if state::get() == FlightState::Armed && rc.throttle > 0.15 {
                // Promote Armed → Flying once the pilot spools up past idle throttle.
                state::set(FlightState::Flying);
            }
        } else if rc.arm && rc.throttle < 0.05 && !rc.failsafe {
            // Pre-arm checks (health_task plausibility gates). A usable IMU is
            // mandatory for any flight; GPS-dependent modes additionally require a
            // nav-ready GPS fix (3-D + bounded accuracy) and a calibrated compass.
            let health = *STATE.sensor_health.lock().await;
            let mode_needs_gps = matches!(rc.mode,
                FlightMode::PositionHold | FlightMode::Auto | FlightMode::ReturnToHome);
            if safety_pin.is_low() {
                defmt::warn!("Arm denied: remove-before-flight pin installed");
            } else if !health.imu_ok {
                defmt::warn!("Arm denied: IMU not healthy");
            } else if mode_needs_gps && !health.gps_ok {
                defmt::warn!("Arm denied: mode requires nav-ready GPS (3D fix + accuracy)");
            } else if mode_needs_gps && !health.mag_ok {
                defmt::warn!("Arm denied: mode requires a calibrated compass");
            } else {
                *STATE.armed.lock().await = true;
                state::set(FlightState::Armed);
                info!("Armed");
            }
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

    // ADC kernel clock from per_ck (HSI). Without it ADC conversions never complete —
    // the reason battery_task was disabled. ⚠ Runtime-unverified: confirm a sane PC0
    // reading and calibrate V_DIVIDER (battery.rs) at hardware bring-up before trusting
    // the low-battery failsafe — a wrong reading reads "critical" and forces RTH/Land.
    config.rcc.mux.adcsel = mux::Adcsel::Per;

    let p = embassy_stm32::init(config);

    init_watchdog();

    // Initialise shared SPI1 bus before spawning — 12.5 MHz, Mode 0 works for
    // both ICM-42688-P (max 24 MHz) and MS5611 (max 20 MHz).
    {
        let mut spi_cfg = spi::Config::default();
        spi_cfg.frequency = Hertz(12_500_000);
        spi_cfg.mode      = spi::MODE_0;
        *SPI1_BUS.lock().await = Some(Spi::new(
            p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH0, Irqs, spi_cfg,
        ));
    }

    let led = Output::new(p.PG7, Level::High, Speed::Low); // active-low: start HIGH = off

    // Remove-before-flight safety pin: 2-pin header on PA0 (Zio) + GND, bridged by a
    // physical pull-pin/jumper. Internal pull-up so "pin removed" (open) reads HIGH =
    // clear to arm; "pin installed" (shorts PA0→GND) reads LOW = arm denied. See the
    // fail-open + arm-transition-only caveats on arming_task's doc comment.
    let safety_pin = Input::new(p.PA0, Pull::Up);

    spawner.spawn(status::led::led_task(led).unwrap());
    spawner.spawn(sensors::imu::imu_task(p.PA4).unwrap());
    #[cfg(not(feature = "pin-test"))]
    spawner.spawn(actuators::motor::motor_task(p.TIM3, p.PB4, p.PB5, p.PB0, p.PB1).unwrap());
    // ⚠ BENCH DIAGNOSTIC (`--features pin-test`): drive the four motor pins as plain GPIO
    // (one HIGH at a time) so a DMM can confirm each STM pin reaches the right ESC pad.
    // No DShot, no TIM3 — nothing spins. TIM3 is simply left unused in this build.
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
    // Battery voltage monitor (ADC3/PC0). The ADC kernel clock is now configured in RCC
    // above (adcsel = PER), so blocking_read no longer wedges the executor. ⚠ Verify the
    // reading + tune V_DIVIDER in battery.rs at hardware bring-up: until calibrated it may
    // read low and trip the critical-battery failsafe (forces RTH/Land). On the bench
    // (no pack on PC0) it reads ~0 V → "critical", harmless since you don't arm-fly there.
    spawner.spawn(sensors::battery::battery_task(p.ADC3, p.PC0).unwrap());
    spawner.spawn(navigation::navigation_task().unwrap());
    spawner.spawn(estimator::estimator_task().unwrap());
    spawner.spawn(health::health_task().unwrap());
    spawner.spawn(control_task().unwrap());
    spawner.spawn(arming_task(safety_pin).unwrap());
    // Flow (MTF-02P): order cancelled — task stays spawned; with nothing on UART4 it
    // simply never sets flow.valid, and every consumer (estimator, land_step) falls
    // back to GPS/baro. Wiring the sensor later needs no firmware change.
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
        // Send a zero DSHOT frame before halting so ESCs receive an explicit stop.
        // Spins ≤30 µs for any in-flight DMA to drain, then fires once and halts.
        actuators::motor::emergency_stop();
        // Turn off status LED (PG7, active-low: set ODR bit to drive HIGH = off).
        // GPIOG base on the STM32H7 is 0x5802_1800 (AHB4). NOTE: this was previously
        // 0x4002_1800 — that's the STM32F4 GPIO base, so on the H723 the write landed
        // in the APB1 region and the LED-off silently did nothing. ODR is at +0x14
        // (offset(5) on a u32 ptr). If the status LED is moved off PG7 (e.g. to an
        // onboard Nucleo LED), update this base/bit to the new port.
        let gpiog = 0x5802_1800 as *mut u32;
        *gpiog.offset(5) = 1 << 7;
    }
    loop {}
}