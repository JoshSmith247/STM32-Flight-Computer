//! LiPo battery voltage monitor - ADC3 on PC0 (pack divider) at 2 Hz.
//! 3 consecutive samples below V_CELL_CRIT latch critical; navigation then forces
//! RTH/Land, and on the ground the task disarms and Fault-locks until pack swap.

use defmt::{info, warn};
use embassy_stm32::{
    adc::{Adc, SampleTime},
    peripherals,
    Peri,
};
use embassy_time::{Duration, Ticker};

use crate::{state, types::BatteryData, STATE};

// Tune these for your hardware

const CELL_COUNT: u32 = 4;          // 3 for 3S, 4 for 4S
const V_DIVIDER:  f32 = 5.7;        // (R1 + R2) / R2
const VREF:       f32 = 3.3;        // STM32 VDDA
const ADC_FULL:   f32 = 4095.0;     // 12-bit resolution

// ESC current sense (CUR pad -> PF3 / ADC3)
// WARNING: Flip to `true` only after the CUR pad is wired AND CUR_A_PER_V is calibrated -
// a floating PF3 reports garbage amps. While false, current_a publishes -1.0.
const CUR_SENSE_FITTED: bool = false;
// A per volt at the CUR pad. 40 A/V is a common BLHeli_S 4-in-1 scale - a
// starting guess ONLY; calibrate before trusting.
const CUR_A_PER_V: f32 = 40.0;

const V_CELL_FULL:  f32 = 4.20;     // 100 %
const V_CELL_EMPTY: f32 = 3.00;     // 0 %
/// Per-cell critical threshold UNDER LOAD. Maps to 25% - must stay BELOW
/// navigation's BATT_LOW_PCT (30%) so low->RTH fires before the critical abort.
const V_CELL_CRIT:  f32 = 3.30;

// Helpers

fn voltage_to_pct(v_total: f32) -> u8 {
    let per_cell = v_total / CELL_COUNT as f32;
    ((per_cell - V_CELL_EMPTY) / (V_CELL_FULL - V_CELL_EMPTY) * 100.0)
        .clamp(0.0, 100.0) as u8
}

// Task

#[embassy_executor::task]
pub async fn battery_task(
    adc_peri: Peri<'static, peripherals::ADC3>,
    mut vbat_pin: Peri<'static, peripherals::PC0>,
    mut cur_pin:  Peri<'static, peripherals::PF3>,
) {
    // no-batt: publish "unknown but OK" once and park; the flight timer is the
    // only pack protection. WARNING: The pilot owns pack health.
    #[cfg(feature = "no-batt")]
    {
        let _ = (&adc_peri, &vbat_pin, &cur_pin);
        *STATE.battery.lock().await =
            BatteryData { voltage_v: 0.0, pct: 0, critical: false, current_a: -1.0 };
        warn!("no-batt build: battery sensing DISABLED — flight timer is the only pack protection");
        core::future::pending::<()>().await;
    }

    let mut adc = Adc::new(adc_peri);

    let mut ticker       = Ticker::every(Duration::from_hz(2));
    // Seeded critical so the first sample can't open a post-boot window where
    // the pre-arm battery gate is open at 0 V. Takes 10 clean samples to clear.
    let mut consec_crit: u8 = 3;
    let mut consec_good: u8 = 0;
    // Log rate-limiting: critical warns every 10th sample, normal info per 10%-decade change.
    let mut crit_log:   u8 = 0;
    let mut log_decade: u8 = 255;

    info!("Battery task started — {}S on PC0, divider {=f32}×", CELL_COUNT, V_DIVIDER);

    loop {
        ticker.next().await;

        let raw    = adc.blocking_read(&mut vbat_pin, SampleTime::Cycles645) as f32;
        let v_adc  = raw / ADC_FULL * VREF;
        let v_batt = v_adc * V_DIVIDER;
        let pct    = voltage_to_pct(v_batt);
        let low    = v_batt / (CELL_COUNT as f32) < V_CELL_CRIT;

        // ESC CUR pad: real pack current when fitted; -1.0 = unknown, telemetry
        // then falls back to its throttle-based estimate.
        let current_a = if CUR_SENSE_FITTED {
            let raw_c = adc.blocking_read(&mut cur_pin, SampleTime::Cycles645) as f32;
            (raw_c / ADC_FULL * VREF) * CUR_A_PER_V
        } else {
            -1.0
        };

        // Asymmetric hysteresis: 3 consecutive lows to go critical, 10 consecutive
        // goods to clear; any bad reading resets the recovery counter.
        if low {
            consec_crit = consec_crit.saturating_add(1);
            consec_good = 0;
        } else {
            consec_good = consec_good.saturating_add(1);
            if consec_good >= 10 { consec_crit = 0; consec_good = 0; }
        }
        let critical = consec_crit >= 3;

        *STATE.battery.lock().await = BatteryData { voltage_v: v_batt, pct, critical, current_a };

        if critical {
            if crit_log == 0 {
                warn!("CRITICAL BATTERY {=f32} V  ({=u8} %)", v_batt, pct);
            }
            crit_log = (crit_log + 1) % 10;

            // Fault-lock once on the ground: arm switch reset required after pack swap.
            // Prefer rangefinder AGL; fall back to baro when flow sensor is invalid.
            let flow  = *STATE.flow.lock().await;
            let armed = *STATE.armed.lock().await;
            let agl   = if flow.usable() && flow.height_mm > 0 && flow.height_mm < 5_000 {
                flow.height_mm as f32 / 1000.0
            } else {
                STATE.baro_data.lock().await.altitude_m
            };
            if armed && agl < 0.15 {
                *STATE.armed.lock().await = false;
                state::set(state::FlightState::Fault);
                warn!("Critical battery — on ground — disarmed, Fault state set");
            }
        } else {
            crit_log = 0;
            if pct / 10 != log_decade {
                info!("Battery {=f32} V  ({=u8} %)", v_batt, pct);
                log_decade = pct / 10;
            }
        }
    }
}
