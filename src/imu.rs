//! ICM-42688-P IMU driver — SPI1 async DMA, 500 Hz sample rate.
//!
//! Pinout (SPI1, AF5):
//!   SPI1_SCK  → PA5   SPI1_MOSI → PA7
//!   SPI1_MISO → PA6   IMU_CS    → PA4 (GPIO, active-low)
//!
//! DMA: DMA2_CH0 (TX), DMA2_CH1 (RX) — both free on this board.
//! ODR set to 1 kHz; task reads at 500 Hz so every read is fresh data.

use core::f32::consts::PI;

use defmt::{error, info};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    peripherals,
    spi::{self, Spi},
    time::Hertz,
    Peri,
};
use embassy_time::{Duration, Ticker, Timer};

use crate::{
    state::FlightState,
    types::{ImuData, Vec3},
    STATE,
};

// ---------------------------------------------------------------------------
// Register map
// ---------------------------------------------------------------------------

const WHO_AM_I:      u8 = 0x75;
const DEVICE_CONFIG: u8 = 0x11;
const PWR_MGMT0:     u8 = 0x4E;
const GYRO_CONFIG0:  u8 = 0x4F;
const ACCEL_CONFIG0: u8 = 0x50;
const TEMP_DATA1:    u8 = 0x1D; // first byte of 14-byte sensor block

const WHO_AM_I_EXPECTED: u8 = 0x47;

// ---------------------------------------------------------------------------
// Scale factors (datasheet §3.1 / §3.2)
// ---------------------------------------------------------------------------

// ±2000 dps full-scale → rad/s per LSB
const GYRO_SCALE:  f32 = (2000.0 / 32768.0) * (PI / 180.0);
// ±16g full-scale → m/s² per LSB
const ACCEL_SCALE: f32 = (16.0 / 32768.0) * 9.80665;

// ---------------------------------------------------------------------------
// Embassy task
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn imu_task(
    spi_peri: Peri<'static, peripherals::SPI1>,
    sck:      Peri<'static, peripherals::PA5>,
    mosi:     Peri<'static, peripherals::PA7>,
    miso:     Peri<'static, peripherals::PA6>,
    cs_pin:   Peri<'static, peripherals::PA4>,
    tx_dma:   Peri<'static, peripherals::DMA2_CH3>,
    rx_dma:   Peri<'static, peripherals::DMA2_CH0>,
    irqs:     crate::Irqs,
) {
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(12_500_000);

    /*let mut spi = Spi::new(spi_peri, sck, mosi, miso, tx_dma, rx_dma, irqs, spi_config);
    let mut cs  = Output::new(cs_pin, Level::High, Speed::High);

    // 1 ms minimum from VDD stable to first SPI transaction (datasheet §6.1).
    Timer::after(Duration::from_millis(10)).await;

    // Soft reset — clears all registers to default state.
    { let mut b = [DEVICE_CONFIG & 0x7F, 0x01]; cs.set_low(); spi.transfer_in_place(&mut b).await.ok(); cs.set_high(); }
    // Device needs up to 1 ms to complete reset before accepting new commands.
    Timer::after(Duration::from_millis(2)).await;

    // Confirm we're talking to the right chip.
    let who = {
        let mut b = [WHO_AM_I | 0x80, 0x00];
        cs.set_low();
        spi.transfer_in_place(&mut b).await.ok();
        cs.set_high();
        b[1]
    };
    if who != WHO_AM_I_EXPECTED {
        error!("ICM-42688-P not found: WHO_AM_I = 0x{:02X} (expected 0x{:02X})", who, WHO_AM_I_EXPECTED);
        crate::state::set(FlightState::Fault);
        loop { Timer::after(Duration::from_secs(1)).await; }
    }
    info!("IMU: ICM-42688-P found (WHO_AM_I = 0x{:02X})", who);

    // PWR_MGMT0: gyro low-noise (bits 3:2 = 11), accel low-noise (bits 1:0 = 11).
    { let mut b = [PWR_MGMT0 & 0x7F, 0x0F]; cs.set_low(); spi.transfer_in_place(&mut b).await.ok(); cs.set_high(); }
    Timer::after(Duration::from_millis(1)).await;

    // GYRO_CONFIG0: FS = ±2000 dps (bits 7:5 = 000), ODR = 1 kHz (bits 3:0 = 0110).
    { let mut b = [GYRO_CONFIG0 & 0x7F, 0x06]; cs.set_low(); spi.transfer_in_place(&mut b).await.ok(); cs.set_high(); }

    // ACCEL_CONFIG0: FS = ±16g (bits 7:5 = 000), ODR = 1 kHz (bits 3:0 = 0110).
    { let mut b = [ACCEL_CONFIG0 & 0x7F, 0x06]; cs.set_low(); spi.transfer_in_place(&mut b).await.ok(); cs.set_high(); }

    // Gyro startup in low-noise mode takes up to 45 ms (datasheet Table 1).
    Timer::after(Duration::from_millis(50)).await;

    info!("IMU: running — ±2000 dps / ±16g @ 1 kHz ODR, sampling at 500 Hz");

    let mut ticker = Ticker::every(Duration::from_hz(500));

    loop {
        ticker.next().await;

        // Burst-read 14 bytes: 2 temp + 6 accel + 6 gyro (registers 0x1D–0x2A).
        // buf[0] = address clocked out; buf[1..15] = data clocked back in.
        // DMA handles the transfer autonomously — CPU is free during the ~10 µs.
        let mut buf = [0u8; 15];
        buf[0] = TEMP_DATA1 | 0x80;
        cs.set_low();
        if spi.transfer_in_place(&mut buf).await.is_err() {
            cs.set_high();
            error!("IMU: SPI transfer error");
            continue;
        }
        cs.set_high();

        // Temperature (°C) — datasheet §4.17: Temp_degC = raw / 132.48 + 25
        let temp_raw = i16::from_be_bytes([buf[1], buf[2]]);
        let temp_c   = (temp_raw as f32 / 132.48) + 25.0;

        // Accelerometer XYZ (m/s²)
        let ax = i16::from_be_bytes([buf[3],  buf[4]])  as f32 * ACCEL_SCALE;
        let ay = i16::from_be_bytes([buf[5],  buf[6]])  as f32 * ACCEL_SCALE;
        let az = i16::from_be_bytes([buf[7],  buf[8]])  as f32 * ACCEL_SCALE;

        // Gyroscope XYZ (rad/s)
        let gx = i16::from_be_bytes([buf[9],  buf[10]]) as f32 * GYRO_SCALE;
        let gy = i16::from_be_bytes([buf[11], buf[12]]) as f32 * GYRO_SCALE;
        let gz = i16::from_be_bytes([buf[13], buf[14]]) as f32 * GYRO_SCALE;

        *STATE.imu_data.lock().await = ImuData {
            accel: Vec3 { x: ax, y: ay, z: az },
            gyro:  Vec3 { x: gx, y: gy, z: gz },
            temp_c,
        };
    }*/
}
