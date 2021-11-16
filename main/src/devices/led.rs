use atsamd_hal::{gpio::{Output, PinId, v2::{Pin, PushPullOutput}}, prelude::*};
use embedded_hal::digital::v2::PinState;

pub struct Led<P: PinId> {
    pin: Pin<P, PushPullOutput>,
    state: PinState
}

impl<P: PinId> Led<P> {
    pub fn init(pin: Pin<P, PushPullOutput>) -> Self {
        Self { pin, state: PinState::Low }
    }

    pub fn on(&mut self) {
        self.pin.set_high();
        self.state = PinState::High;
    }
    pub fn off(&mut self) {
        self.pin.set_low();
        self.state = PinState::Low;
    }
    pub fn toggle(&mut self) {
        self.state = !self.state;
        self.pin.set_state(self.state);
    }
}
