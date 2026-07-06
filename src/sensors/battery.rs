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

const CELL_COUNT: u32 = 4;          // 3 for 3S, 4 for 4S
const V_DIVIDER:  f32 = 5.7;        // (R1 + R2) / R2
const VREF:       f32 = 3.3;        // STM32 VDDA
const ADC_FULL:   f32 = 4095.0;     // 12-bit resolution

// ── ESC current sense (CUR pad → PF3 / ADC3) ────────────────────────────────
// ⚠ Flip to `true` only after the CUR pad is wired AND CUR_A_PER_V is calibrated
// against a known load (e.g. bench supply current limit or a clamp meter).
// While false, current_a publishes -1.0 and telemetry falls back to its
// throttle-based estimate — a floating PF3 would otherwise report garbage amps.
const CUR_SENSE_FITTED: bool = false;
// A per volt at the CUR pad. 40 A/V (25 mV/A) is a common BLHeli_S 4-in-1 scale
// (Betaflight "scale 250") — a starting guess ONLY; calibrate before trusting.
const CUR_A_PER_V: f32 = 40.0;

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
    mut cur_pin:  Peri<'static, peripherals::PF3>,
) {
    let mut adc = Adc::new(adc_peri);

    let mut ticker       = Ticker::every(Duration::from_hz(2));
    let mut consec_crit: u8 = 0;
    let mut consec_good: u8 = 0;

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

        // Asymmetric hysteresis: 3 consecutive low samples (~1.5 s) to go
        // critical; 10 consecutive good samples (~5 s) without any low sample
        // to clear it. Any bad reading resets the recovery counter.
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
            warn!("CRITICAL BATTERY {=f32} V  ({=u8} %)", v_batt, pct);

            // Fault-lock once on the ground: arm switch reset required after pack swap.
            // Prefer rangefinder AGL; fall back to baro when flow sensor is invalid.
            let flow  = *STATE.flow.lock().await;
            let armed = *STATE.armed.lock().await;
            let agl   = if flow.valid && flow.height_mm > 0 && flow.height_mm < 5_000 {
                flow.height_mm as f32 / 1000.0
            } else {
                STATE.baro_data.lock().await.altitude_m
            };
            if armed && agl < 0.15 {
                *STATE.armed.lock().await = false;
                state::set(state::FlightState::Fault);
                warn!("Critical battery — on ground — disarmed, Fault state set");
            }
        } else if pct % 10 == 0 {
            info!("Battery {=f32} V  ({=u8} %)", v_batt, pct);
        }
    }
}
