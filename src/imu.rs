//! IMU driver — ICM-42688-P over SPI1 at 24 MHz.
//!
//! Runs at 2000 Hz to feed the AHRS filter.
//! After reading raw accel + gyro, it calls `ahrs::update()` and writes
//! the resulting quaternion into `STATE.attitude`.

use embassy_stm32::{
    gpio::{Level, Output, Speed},
    peripherals,
    spi::{Config as SpiConfig, Spi},
};
use embassy_time::{Duration, Ticker};
use defmt::info;

use crate::{ahrs, types::{ImuData, Vec3}, STATE};

// ICM-42688-P register addresses
const REG_WHO_AM_I:      u8 = 0x75;
const REG_PWR_MGMT0:     u8 = 0x4E;
const REG_GYRO_CONFIG0:  u8 = 0x4F;
const REG_ACCEL_CONFIG0: u8 = 0x50;
const REG_ACCEL_DATA_X1: u8 = 0x1F; // first of 12 bytes: accel XYZ + gyro XYZ (16-bit each)

const WHO_AM_I_EXPECTED: u8 = 0x47;

// Scale factors
const ACCEL_SCALE: f32 = 9.81 / 2048.0;  // ±16 g range → m/s²
const GYRO_SCALE:  f32 = 0.001065;        // ±2000 dps range → rad/s  (π/180/16.4)

/// SPI peripheral + CS pin types for STM32F405 SPI1.
type ImuSpi = Spi<'static, peripherals::SPI1, peripherals::DMA2_CH3, peripherals::DMA2_CH0>;
type ImuCs = Output<'static, peripherals::PA4>;

pub struct Icm42688p {
    spi: ImuSpi,
    cs:  ImuCs,
}

impl Icm42688p {
    pub async fn new(spi: ImuSpi, cs_pin: ImuCs) -> Self {
        let mut dev = Self { spi, cs: cs_pin };
        dev.init().await;
        dev
    }

    async fn init(&mut self) {
        // Soft-reset
        self.write_reg(0x11, 0x01).await;
        embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;

        let who = self.read_reg(REG_WHO_AM_I).await;
        assert_eq!(who, WHO_AM_I_EXPECTED, "ICM-42688-P not found (got 0x{:02X})", who);

        // Enable accel + gyro in low-noise mode
        self.write_reg(REG_PWR_MGMT0, 0x0F).await;
        embassy_time::Timer::after(embassy_time::Duration::from_micros(300)).await;

        // Gyro: ±2000 dps, 2 kHz ODR
        self.write_reg(REG_GYRO_CONFIG0, 0x00).await;
        // Accel: ±16 g, 2 kHz ODR
        self.write_reg(REG_ACCEL_CONFIG0, 0x00).await;

        info!("ICM-42688-P initialised");
    }

    pub async fn read_sample(&mut self) -> ImuData {
        // Burst-read 12 bytes starting at ACCEL_DATA_X1
        let mut buf = [0u8; 13]; // 1 address byte + 12 data bytes
        buf[0] = REG_ACCEL_DATA_X1 | 0x80; // read bit

        self.cs.set_low();
        self.spi.transfer_in_place(&mut buf).await.unwrap();
        self.cs.set_high();

        let ax = i16::from_be_bytes([buf[1],  buf[2]])  as f32 * ACCEL_SCALE;
        let ay = i16::from_be_bytes([buf[3],  buf[4]])  as f32 * ACCEL_SCALE;
        let az = i16::from_be_bytes([buf[5],  buf[6]])  as f32 * ACCEL_SCALE;
        let gx = i16::from_be_bytes([buf[7],  buf[8]])  as f32 * GYRO_SCALE;
        let gy = i16::from_be_bytes([buf[9],  buf[10]]) as f32 * GYRO_SCALE;
        let gz = i16::from_be_bytes([buf[11], buf[12]]) as f32 * GYRO_SCALE;

        ImuData {
            accel: Vec3 { x: ax, y: ay, z: az },
            gyro:  Vec3 { x: gx, y: gy, z: gz },
            temp_c: 0.0, // TODO: read temp register
        }
    }

    async fn write_reg(&mut self, reg: u8, val: u8) {
        let mut buf = [reg & 0x7F, val];
        self.cs.set_low();
        self.spi.transfer_in_place(&mut buf).await.unwrap();
        self.cs.set_high();
    }

    async fn read_reg(&mut self, reg: u8) -> u8 {
        let mut buf = [reg | 0x80, 0x00];
        self.cs.set_low();
        self.spi.transfer_in_place(&mut buf).await.unwrap();
        self.cs.set_high();
        buf[1]
    }
}

// ---------------------------------------------------------------------------
// Embassy task — 2000 Hz IMU read + AHRS update
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn imu_task(
    spi_peri: peripherals::SPI1,
    cs:       peripherals::PA4,
    sck:      peripherals::PA5,
    miso:     peripherals::PA6,
    mosi:     peripherals::PA7,
    tx_dma:   peripherals::DMA2_CH3,
    rx_dma:   peripherals::DMA2_CH0,
) {
    use embassy_stm32::spi::BitOrder;

    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency       = embassy_stm32::time::Hertz(24_000_000);
    spi_cfg.bit_order       = BitOrder::MsbFirst;

    let spi  = Spi::new(spi_peri, sck, mosi, miso, tx_dma, rx_dma, spi_cfg);
    let cs   = Output::new(cs, Level::High, Speed::VeryHigh);
    let mut imu = Icm42688p::new(spi, cs).await;

    let mut ahrs = ahrs::MadgwickFilter::new(0.1, 0.0005); // beta, sample_period
    let mut ticker = Ticker::every(Duration::from_hz(2000));

    loop {
        ticker.next().await;

        let sample = imu.read_sample().await;
        *STATE.imu_data.lock().await = sample;

        // Run Madgwick filter → quaternion attitude estimate
        let q = ahrs.update(sample.gyro, sample.accel);
        *STATE.attitude.lock().await = q;
    }
}
