#![no_std]
#![no_main]

mod ahrs;
mod navigation;
mod pid;
mod state;
mod telemetry;
mod types;

mod sensors;    // baro, battery, flow, gps, imu, mag, rc
mod actuators;  // motor, servo
mod status;     // led

use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, gpio::{Level, Output, Speed}, mode::Async, spi::{self, Spi}, time::Hertz};
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
    use pid::{mix_quad_x, FlightPids};

    let mut filter = MadgwickFilter::new(0.1, 1.0 / 500.0);
    let mut pids = FlightPids::default();
    let mut ticker = Ticker::every(Duration::from_hz(500));
    let mut wdg_tick: u32 = 0;

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

        if !is_armed || rc.failsafe {
            pids.reset_all();
            *STATE.motor_outputs.lock().await = Default::default();
            continue;
        }

        let (roll_out, pitch_out, yaw_out) = pids.update(&setpoint, &euler, imu.gyro);
        let outputs = mix_quad_x(setpoint.throttle, roll_out, pitch_out, yaw_out);
        *STATE.motor_outputs.lock().await = outputs;
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
#[embassy_executor::task]
async fn arming_task() {
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
            // Reject arm in GPS-dependent modes without a 3-D fix.
            let mode_needs_gps = matches!(rc.mode,
                FlightMode::PositionHold | FlightMode::Auto | FlightMode::ReturnToHome);
            if mode_needs_gps && !STATE.gps_fix.lock().await.fix_ok {
                defmt::warn!("Arm denied: mode requires GPS 3D fix");
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

    spawner.spawn(status::led::led_task(led).unwrap());
    spawner.spawn(sensors::imu::imu_task(p.PA4).unwrap());
    spawner.spawn(actuators::motor::motor_task(p.TIM3, p.PB4, p.PB5, p.PB0, p.PB1).unwrap());
    spawner.spawn(sensors::rc::rc_task(p.USART2, p.PA2, p.PA3, p.DMA1_CH5, Irqs).unwrap());
    spawner.spawn(telemetry::telemetry_task(p.USART3, p.PB11, p.PB10, p.DMA1_CH3, p.DMA1_CH1, Irqs).unwrap());
    spawner.spawn(sensors::baro::baro_task(p.PA8).unwrap());
    spawner.spawn(sensors::gps::gps_task(p.USART1, p.PA10, p.PA9, p.DMA2_CH6, p.DMA2_CH5, Irqs).unwrap());
    // DISABLED: battery_task's `adc.blocking_read()` busy-spins forever because the
    // ADC3 kernel clock is not configured in the RCC setup above — wedging the
    // single-threaded executor (board appears dead). Re-enable once the ADC clock
    // is set (e.g. config.rcc.mux.adcsel) or the read is moved to async DMA.
    // Not needed for bench motor testing. See sensors/battery.rs:66.
    // spawner.spawn(sensors::battery::battery_task(p.ADC3, p.PC0).unwrap());
    spawner.spawn(navigation::navigation_task().unwrap());
    spawner.spawn(control_task().unwrap());
    spawner.spawn(arming_task().unwrap());
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