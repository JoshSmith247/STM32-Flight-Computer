use core::sync::atomic::{AtomicU8, Ordering};

use crate::motors;

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

pub fn set(state: FlightState) {
    FLIGHT_STATE.store(state as u8, Ordering::Relaxed);

    if state == FlightState::Arming {
        // Perform arming actions
        arm();
        set(FlightState::Armed);
    }

    defmt::info!("Flight state -> {}", state);
}

fn arm() {
    // Initialize connection to remote control
    // Initialize nav
    // Start motors (aprox. 10 sec test before flight)
    //motors::motors_task()
}