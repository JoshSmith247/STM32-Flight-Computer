#![no_std]
#![no_main]

mod ahrs;
mod baro;
mod battery;
mod gps;
mod imu;
mod led;
mod motor;
mod navigation;
mod pid;
mod rc;
mod state;
mod telemetry;
mod types;

use embassy_executor::Spawner;
use embassy_stm32::{bind_interrupts, gpio::{Level, Output, Speed}, mode::Async, spi::{self, Spi}, time::Hertz};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker, Timer};
use state::FlightState;
use defmt::info;
use {defmt_rtt as _};

pub static STATE: types::SharedState = types::SharedState::new();

/// Shared SPI1 bus — IMU (CS=PA4) and baro (CS=PA8) are both on this bus.
/// Initialised in main() before any task is spawned.
pub type SpiBus = Spi<'static, Async, spi::mode::Master>;
pub static SPI1_BUS: Mutex<CriticalSectionRawMutex, Option<SpiBus>> = Mutex::new(None);

bind_interrupts!(pub struct Irqs {
    USART1       => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
    USART2       => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
    USART3       => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART3>;
    //SPI1       => embassy_stm32::spi::InterruptHandler<embassy_stm32::peripherals::SPI1>;
    DMA1_STREAM1 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH1>;
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

    loop {
        ticker.next().await;

        let imu      = *STATE.imu_data.lock().await;
        let is_armed = *STATE.armed.lock().await;
        let nav_cmd  = *STATE.nav_command.lock().await;
        let rc       = *STATE.rc_input.lock().await;

        let setpoint = if nav_cmd.autonomous {
            nav_cmd.attitude_setpoint
        } else {
            rc.to_attitude_setpoint()
        };

        let quat = filter.update(imu.gyro, imu.accel);
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
            }
        } else if rc.arm && rc.throttle < 0.05 && !rc.failsafe {
            *STATE.armed.lock().await = true;
            state::set(FlightState::Flying);
            info!("Armed");
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

    spawner.spawn(led::led_task(led).unwrap());
    spawner.spawn(imu::imu_task(p.PA4).unwrap());
    spawner.spawn(motor::motor_task(p.TIM3, p.PB4, p.PB5, p.PB0, p.PB1).unwrap());
    spawner.spawn(rc::rc_task(p.USART2, p.PA2, p.PA3, p.DMA1_CH5, Irqs).unwrap());
    spawner.spawn(telemetry::telemetry_task(p.USART3, p.PB11, p.PB10, p.DMA1_CH3, p.DMA1_CH1, Irqs).unwrap());
    spawner.spawn(baro::baro_task(p.PA8).unwrap());
    spawner.spawn(gps::gps_task(p.USART1, p.PA10, p.PA9, p.DMA2_CH6, p.DMA2_CH5, Irqs).unwrap());
    spawner.spawn(battery::battery_task(p.ADC3, p.PC0).unwrap());
    spawner.spawn(navigation::navigation_task().unwrap());
    spawner.spawn(control_task().unwrap());
    spawner.spawn(arming_task().unwrap());

    state::set(FlightState::Idle);

    loop {
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // FORCE PORT G PIN 7 LOW TO TURN OFF THE LED
    unsafe {
        // This directly writes to the hardware register to turn off PG7
        let gpiog = 0x40021800 as *mut u32;
        *gpiog.offset(5) = 1 << 7;
    }
    loop {}
}