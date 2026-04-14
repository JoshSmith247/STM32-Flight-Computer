//! STM32F405 Autonomous Drone Flight Computer
//!
//! Hardware targets:
//!   MCU:  STM32F405RGT6 (168 MHz Cortex-M4F)
//!   IMU:  ICM-42688-P   (SPI1)
//!   Baro: MS5611        (SPI1, separate CS)
//!   GPS:  u-blox M8N   (USART1)
//!   RC:   SBUS          (USART2, inverted)
//!   ESC:  DSHOT600      (TIM3, 4 channels)
//!   Telem:MAVLink       (USART3)
//!
//! Task layout (Embassy executor threads):
//!   @2000 Hz  imu_task        – read IMU, run AHRS filter
//!   @500  Hz  control_task    – PID loops (rate → attitude → position)
//!   @100  Hz  navigation_task – waypoint sequencer / autonomous modes
//!   @100  Hz  telemetry_task  – MAVLink heartbeat + sensor stream
//!   @50   Hz  baro_task       – barometer read + altitude estimate
//!   IRQ-driven rc_task        – SBUS frame parser

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    peripherals,
    usart::{self, Uart},
    Config,
};
use panic_probe as _;

mod ahrs;
mod baro;
mod imu;
mod motor;
mod navigation;
mod pid;
mod rc;
mod telemetry;
mod types;

use types::SharedState;

// Bind STM32 interrupts to Embassy handlers
bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

/// Global shared state passed between tasks via Embassy channels/mutexes.
/// See `types.rs` for full definitions.
static STATE: SharedState = SharedState::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // --- Clocks & peripheral init ---
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse { freq: embassy_stm32::time::Hertz(8_000_000), mode: HseMode::Oscillator });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2),  // SYSCLK = 168 MHz
            divq: Some(PllQDiv::DIV7),  // USB = 48 MHz
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre  = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
    }
    let p = embassy_stm32::init(config);

    defmt::info!("Flight computer booting — STM32F405 @ 168 MHz");

    // --- Spawn tasks ---
    spawner.must_spawn(imu::imu_task(p.SPI1, p.PA4, p.PA5, p.PA6, p.PA7, p.DMA2_CH3, p.DMA2_CH0));
    spawner.must_spawn(baro::baro_task(p.PA8));         // SPI CS for MS5611
    spawner.must_spawn(rc::rc_task(p.USART2, p.PA2, p.PA3, p.DMA1_CH5, Irqs));
    spawner.must_spawn(telemetry::telemetry_task(p.USART3, p.PB10, p.PB11, p.DMA1_CH3, Irqs));
    spawner.must_spawn(navigation::navigation_task());
    spawner.must_spawn(motor::motor_task(p.TIM3, p.PB4, p.PB5, p.PB0, p.PB1));
    spawner.must_spawn(control_task());

    defmt::info!("All tasks spawned");
}

/// 500 Hz PID control loop — reads AHRS output, RC setpoints, runs PID cascade.
#[embassy_executor::task]
async fn control_task() {
    use embassy_time::{Duration, Ticker};
    let mut ticker = Ticker::every(Duration::from_hz(500));

    loop {
        ticker.next().await;

        let attitude  = STATE.attitude.lock().await;
        let rc_input  = STATE.rc_input.lock().await;
        let nav_cmd   = STATE.nav_command.lock().await;

        // Select manual vs autonomous setpoints
        let setpoint = if nav_cmd.autonomous {
            nav_cmd.attitude_setpoint
        } else {
            rc_input.to_attitude_setpoint()
        };

        // Run full PID cascade (rate → attitude → [position])
        // TODO: call pid::cascade_update(...)
        // TODO: feed motor_task via STATE.motor_outputs
        drop((attitude, rc_input, nav_cmd, setpoint));
    }
}
