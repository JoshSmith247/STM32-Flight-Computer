//! Barometer driver — MS5611 over SPI (shared SPI1 bus, separate CS).
//!
//! The MS5611 requires oversampling: each conversion takes 0.5–10 ms
//! depending on OSR. We use OSR=4096 (8.22 ms) for maximum accuracy.
//! Reads are interleaved: trigger pressure → wait → read, trigger temp → wait → read.
//!
//! Output: pressure (Pa) + temperature (°C) → altitude estimate via ISA formula.

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Ticker, Timer};
use defmt::info;
use libm::logf;

use crate::{types::BaroData, STATE};

// MS5611 SPI commands
const CMD_RESET:       u8 = 0x1E;
const CMD_PROM_READ:   u8 = 0xA0; // + 2*i for coefficients C1–C6
const CMD_CONV_D1:     u8 = 0x48; // pressure OSR=4096
const CMD_CONV_D2:     u8 = 0x58; // temperature OSR=4096
const CMD_ADC_READ:    u8 = 0x00;

/// Conversion time for OSR=4096
const CONV_TIME: Duration = Duration::from_millis(10);

/// ISA altitude from pressure (Pa). Reference: https://en.wikipedia.org/wiki/Barometric_formula
/// Uses simple troposphere model valid 0–11 km.
fn pressure_to_altitude_m(pressure_pa: f32, sea_level_pa: f32) -> f32 {
    // h = 44330 * (1 - (p/p0)^(1/5.255))
    44330.0 * (1.0 - libm::powf(pressure_pa / sea_level_pa, 0.1902949))
}

// ---------------------------------------------------------------------------
// Embassy task — 50 Hz barometer read
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn baro_task(
    // CS pin for MS5611 on shared SPI1 bus
    // In a real implementation, share the SPI bus via a mutex in STATE.
    cs_pin: embassy_stm32::peripherals::PA8,
) {
    // TODO: initialise SPI access via shared bus mutex
    // TODO: read 6 PROM calibration coefficients (C1–C6) after reset
    info!("Baro task started (MS5611 stub — SPI bus sharing TODO)");

    // Placeholder calibration coefficients (must be read from PROM on real hardware)
    let c1: u16 = 40127; // Pressure sensitivity
    let c2: u16 = 36924; // Pressure offset
    let c3: u16 = 23317; // TCS
    let c4: u16 = 23282; // TCO
    let c5: u16 = 33464; // T_REF
    let c6: u16 = 28312; // TEMPSENS

    let sea_level_pa: f32 = 101325.0;
    let mut ticker = Ticker::every(Duration::from_hz(50));

    loop {
        ticker.next().await;

        // TODO: trigger D1 (pressure) conversion via SPI, wait CONV_TIME, read ADC
        // TODO: trigger D2 (temperature) conversion, wait, read ADC
        let d1: u32 = 9085466; // raw pressure ADC (placeholder)
        let d2: u32 = 8569150; // raw temperature ADC (placeholder)

        // --- MS5611 second-order compensation (from datasheet) ---
        let dt    = d2 as i32 - (c5 as i32) * 256;
        let temp  = 2000 + (dt as i64 * c6 as i64 / (1 << 23)) as i32;

        let off   = (c2 as i64) * 65536 + (c4 as i64 * dt as i64) / 128;
        let sens  = (c1 as i64) * 32768 + (c3 as i64 * dt as i64) / 256;

        // Second-order corrections (below 20°C)
        let (off2, sens2) = if temp < 2000 {
            let t2    = (dt as i64 * dt as i64) >> 31;
            let off2  = 5 * (temp as i64 - 2000).pow(2) / 2;
            let sens2 = 5 * (temp as i64 - 2000).pow(2) / 4;
            (off2, sens2)
        } else {
            (0i64, 0i64)
        };

        let pressure_pa = ((d1 as i64 * (sens - sens2) / (1 << 21) - (off - off2)) / (1 << 15)) as f32;
        let temp_c      = (temp as f32) / 100.0;
        let altitude_m  = pressure_to_altitude_m(pressure_pa, sea_level_pa);

        *STATE.baro_data.lock().await = BaroData { pressure_pa, temp_c, altitude_m };
    }
}
