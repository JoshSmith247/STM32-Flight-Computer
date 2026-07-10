//! MS5611 barometer driver - SPI1 shared bus (with the IMU), CS on PA8, 25 Hz.
//! D1/D2 conversions with second-order compensation (AN520), altitude via ISA model.

use defmt::{error, info, warn};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    peripherals,
    Peri,
};
use embassy_time::{Duration, Ticker, Timer};

use crate::{
    types::BaroData,
    STATE,
};

// MS5611 SPI commands

const CMD_RESET:     u8 = 0x1E;
const CMD_PROM_READ: u8 = 0xA0; // base; C1-C6 at +0x02, +0x04 ... +0x0C
const CMD_CONV_D1:   u8 = 0x48; // pressure, OSR=4096
const CMD_CONV_D2:   u8 = 0x58; // temperature, OSR=4096
const CMD_ADC_READ:  u8 = 0x00;

const CONV_DELAY: Duration = Duration::from_millis(10); // OSR=4096 -> 8.22 ms max

// SPI transaction helpers

/// Send a command byte with no response data (e.g. reset, start conversion).
async fn cmd(cs: &mut Output<'_>, command: u8) {
    let mut buf = [command];
    let mut bus = crate::SPI1_BUS.lock().await;
    let Some(spi) = bus.as_mut() else { return Default::default(); };
    cs.set_low();
    spi.transfer_in_place(&mut buf).await.ok();
    cs.set_high();
}

/// Read a 16-bit PROM word. `prom_addr` is the full command byte (0xA2-0xAC).
async fn prom_read(cs: &mut Output<'_>, prom_addr: u8) -> u16 {
    let mut buf = [prom_addr, 0x00, 0x00];
    let mut bus = crate::SPI1_BUS.lock().await;
    let Some(spi) = bus.as_mut() else { return Default::default(); };
    cs.set_low();
    spi.transfer_in_place(&mut buf).await.ok();
    cs.set_high();
    u16::from_be_bytes([buf[1], buf[2]])
}

/// Read the 24-bit ADC result after a conversion has completed.
async fn adc_read(cs: &mut Output<'_>) -> u32 {
    let mut buf = [CMD_ADC_READ, 0x00, 0x00, 0x00];
    let mut bus = crate::SPI1_BUS.lock().await;
    let Some(spi) = bus.as_mut() else { return Default::default(); };
    cs.set_low();
    spi.transfer_in_place(&mut buf).await.ok();
    cs.set_high();
    u32::from_be_bytes([0, buf[1], buf[2], buf[3]])
}

// Altitude from pressure, ISA troposphere model. `ref_pa` is the zero-altitude
// datum: passing the captured ground pressure yields height above launch, NOT MSL.
fn pressure_to_altitude_m(pressure_pa: f32, ref_pa: f32) -> f32 {
    44330.0 * (1.0 - libm::powf(pressure_pa / ref_pa, 0.190_294))
}

// Embassy task

#[embassy_executor::task]
pub async fn baro_task(cs_pin: Peri<'static, peripherals::PA8>) {
    let mut cs = Output::new(cs_pin, Level::High, Speed::High);

    // MS5611 needs VDD stable for at least 2.8 ms before first command.
    Timer::after(Duration::from_millis(5)).await;

    // Reset: reloads factory calibration from PROM into the ADC registers.
    cmd(&mut cs, CMD_RESET).await;
    Timer::after(Duration::from_millis(5)).await; // 2.8 ms reload time + margin

    // Read factory calibration coefficients C1-C6 from PROM (0xA2..0xAC).
    let c1 = prom_read(&mut cs, CMD_PROM_READ | 0x02).await;
    let c2 = prom_read(&mut cs, CMD_PROM_READ | 0x04).await;
    let c3 = prom_read(&mut cs, CMD_PROM_READ | 0x06).await;
    let c4 = prom_read(&mut cs, CMD_PROM_READ | 0x08).await;
    let c5 = prom_read(&mut cs, CMD_PROM_READ | 0x0A).await;
    let c6 = prom_read(&mut cs, CMD_PROM_READ | 0x0C).await;

    // All-zero coefficients means the PROM read failed (wiring / CS issue).
    // Non-fatal: barometer is not required for Stabilize mode; log and park the task.
    if c1 == 0 && c2 == 0 && c3 == 0 {
        error!("Baro: PROM read returned all zeros — check SPI wiring and CS pin (continuing without baro)");
        loop { Timer::after(Duration::from_secs(10)).await; }
    }

    info!("Baro: MS5611 ready — C1={} C2={} C3={} C4={} C5={} C6={}", c1, c2, c3, c4, c5, c6);

    // Ground-pressure reference averaged over the first 30 valid samples. DATUM:
    // published altitude_m is relative-to-launch, NOT MSL - touchdown thresholds depend on this.
    const GROUND_AVG_SAMPLES: u32 = 30;
    let mut ground_pa:      f32 = 0.0;
    let mut ground_samples: u32 = 0;

    // 25 Hz - each cycle spends ~20 ms awaiting conversions, fitting neatly
    // inside the 40 ms tick. The SPI bus is free (mutex released) during waits.
    let mut ticker = Ticker::every(Duration::from_hz(25));

    loop {
        ticker.next().await;

        // --- D1: pressure conversion ---
        cmd(&mut cs, CMD_CONV_D1).await;
        Timer::after(CONV_DELAY).await;
        let d1 = adc_read(&mut cs).await;

        // --- D2: temperature conversion ---
        cmd(&mut cs, CMD_CONV_D2).await;
        Timer::after(CONV_DELAY).await;
        let d2 = adc_read(&mut cs).await;

        if d1 == 0 && d2 == 0 {
            warn!("Baro: ADC returned zeros — skipping");
            continue;
        }

        // --- MS5611 second-order compensation (datasheet AN520, 4.4) ---

        // dT: difference between actual and reference temperature
        let dt = d2 as i32 - (c5 as i32) * 256;

        // First-order temperature (hundredths of degC)
        let mut temp: i32 = 2000 + (dt as i64 * c6 as i64 / (1 << 23)) as i32;

        // First-order pressure offset and sensitivity
        let mut off:  i64 = (c2 as i64) * 65536 + (c4 as i64 * dt as i64) / 128;
        let mut sens: i64 = (c1 as i64) * 32768 + (c3 as i64 * dt as i64) / 256;

        // Second-order corrections for temperatures below 20 degC
        if temp < 2000 {
            let t2    = (dt as i64 * dt as i64) >> 31;
            let off2  = 5 * (temp as i64 - 2000).pow(2) / 2;
            let sens2 = 5 * (temp as i64 - 2000).pow(2) / 4;
            temp -= t2 as i32;
            off  -= off2;
            sens -= sens2;
        }

        let pressure_pa = ((d1 as i64 * sens / 2_097_152 - off) / 32_768) as f32;
        let temp_c      = temp as f32 / 100.0;

        // Build the ground reference from the first GROUND_AVG_SAMPLES valid
        // readings; report 0 m AGL until it is established.
        if ground_samples < GROUND_AVG_SAMPLES {
            ground_pa      += pressure_pa;
            ground_samples += 1;
            if ground_samples == GROUND_AVG_SAMPLES {
                ground_pa /= GROUND_AVG_SAMPLES as f32;
                info!(
                    "Baro: ground reference {} Pa ({} samples) — altitude_m now relative-to-launch (AGL, not MSL)",
                    ground_pa as u32, GROUND_AVG_SAMPLES
                );
            }
            *STATE.baro_data.lock().await = BaroData { pressure_pa, temp_c, altitude_m: 0.0 };
            continue;
        }

        let altitude_m = pressure_to_altitude_m(pressure_pa, ground_pa);
        *STATE.baro_data.lock().await = BaroData { pressure_pa, temp_c, altitude_m };
    }
}
