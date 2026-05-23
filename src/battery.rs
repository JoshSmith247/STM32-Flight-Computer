//! LiPo battery voltage monitor — ADC3 on PC0 at 2 Hz.
//!
//! Hardware: voltage divider between pack positive and GND, midpoint → PC0.
//!   Example: R1 = 4.7 kΩ (pack side), R2 = 1 kΩ (GND side)
//!   → V_DIVIDER = (4.7 + 1.0) / 1.0 = 5.7
//!
//! Tune CELL_COUNT and V_DIVIDER for your pack and resistor values.
//!
//! Safe-abort behaviour (navigation_task reads STATE.battery.critical):
//!   • 5 consecutive samples below V_CELL_CRIT (≈ 2.5 s) → critical = true
//!   • Navigation overrides to RTH (GPS fix present) or Land (no fix)
//!   • Once on the ground (baro alt < 0.3 m) the task disarms and sets Fault
//!     state, preventing re-arm until the battery is swapped
//!
//! NOTE: The embassy-stm32 ADC API changes between versions.  If Adc::new
//! requires a Delay argument use: Adc::new(adc_peri, &mut embassy_time::Delay).
//! If blocking_read is not available, replace it with `.read(&mut pin).await`.

use defmt::{info, warn};
use embassy_stm32::{
    adc::{Adc, SampleTime},
    peripherals,
    Peri,
};
use embassy_time::{Duration, Ticker};

use crate::{state, types::BatteryData, STATE};

// ── Tune these for your hardware ────────────────────────────────────────────

const CELL_COUNT: u32 = 3;          // 3 for 3S, 4 for 4S
const V_DIVIDER:  f32 = 5.7;        // (R1 + R2) / R2
const VREF:       f32 = 3.3;        // STM32 VDDA
const ADC_FULL:   f32 = 4095.0;     // 12-bit resolution

const V_CELL_FULL:  f32 = 4.20;     // 100 %
const V_CELL_EMPTY: f32 = 3.00;     //   0 %
const V_CELL_CRIT:  f32 = 3.50;     //  ~5 % — triggers safe abort

// ── Helpers ─────────────────────────────────────────────────────────────────

fn voltage_to_pct(v_total: f32) -> u8 {
    let per_cell = v_total / CELL_COUNT as f32;
    ((per_cell - V_CELL_EMPTY) / (V_CELL_FULL - V_CELL_EMPTY) * 100.0)
        .clamp(0.0, 100.0) as u8
}

// ── Task ─────────────────────────────────────────────────────────────────────

#[embassy_executor::task]
pub async fn battery_task(
    adc_peri: Peri<'static, peripherals::ADC3>,
    mut vbat_pin: Peri<'static, peripherals::PC0>,
) {
    let mut adc = Adc::new(adc_peri);

    let mut ticker       = Ticker::every(Duration::from_hz(2));
    let mut consec_crit: u8 = 0;

    info!("Battery task started — {}S on PC0, divider {=f32}×", CELL_COUNT, V_DIVIDER);

    loop {
        ticker.next().await;

        let raw    = adc.blocking_read(&mut vbat_pin, SampleTime::Cycles645) as f32;
        let v_adc  = raw / ADC_FULL * VREF;
        let v_batt = v_adc * V_DIVIDER;
        let pct    = voltage_to_pct(v_batt);
        let low    = v_batt / (CELL_COUNT as f32) < V_CELL_CRIT;

        // Symmetric hysteresis: 5 consecutive low samples to go critical,
        // 5 consecutive good samples to clear it.
        if low { consec_crit = consec_crit.saturating_add(1); }
        else   { consec_crit = consec_crit.saturating_sub(1); }
        let critical = consec_crit >= 5;

        *STATE.battery.lock().await = BatteryData { voltage_v: v_batt, pct, critical };

        if critical {
            warn!("CRITICAL BATTERY {=f32} V  ({=u8} %)", v_batt, pct);

            // Fault-lock once on the ground: arm switch reset required after pack swap
            let alt   = STATE.baro_data.lock().await.altitude_m;
            let armed = *STATE.armed.lock().await;
            if armed && alt < 0.3 {
                *STATE.armed.lock().await = false;
                state::set(state::FlightState::Fault);
                warn!("Critical battery — on ground — disarmed, Fault state set");
            }
        } else if pct % 10 == 0 {
            info!("Battery {=f32} V  ({=u8} %)", v_batt, pct);
        }
    }
}
