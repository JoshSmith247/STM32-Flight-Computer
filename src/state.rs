use core::sync::atomic::{AtomicU8, Ordering};

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum FlightState {
    Idle = 0,
    Arming = 1,
    Armed = 2,
    Flying = 3,
    Landing = 4,
    Fault = 5,
}

impl FlightState {
    fn from_u8(val: u8) -> Self {
        match val {
            1 => Self::Arming,
            2 => Self::Armed,
            3 => Self::Flying,
            4 => Self::Landing,
            5 => Self::Fault,
            _ => Self::Idle,
        }
    }
}

static FLIGHT_STATE: AtomicU8 = AtomicU8::new(0);

pub fn get() -> FlightState {
    FlightState::from_u8(FLIGHT_STATE.load(Ordering::Relaxed))
}

/// Set the flight state. Re-setting the current state is a silent no-op -
/// 100 Hz failsafe paths call this every tick while a condition holds.
pub fn set(state: FlightState) {
    let prev = FLIGHT_STATE.swap(state as u8, Ordering::Relaxed);
    if prev != state as u8 {
        defmt::info!("Flight state -> {}", state);
    }
}