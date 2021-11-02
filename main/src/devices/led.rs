use atsamd_hal::gpio::PinId;
use xiao_m0::{
    gpio::{Output, Pin, PushPull},
    prelude::*,
};

pub struct Led<P: PinId> {
    pin: Pin<P, Output<PushPull>>,
}

impl<P: PinId> Led<P> {
    pub fn init(pin: Pin<P, Output<PushPull>>) -> Self {
        Self { pin }
    }

    pub fn on(&mut self) {
        self.pin.set_high();
    }
    pub fn off(&mut self) {
        self.pin.set_low();
    }
}
