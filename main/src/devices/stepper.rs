use xiao_m0::{
    gpio::{
        v2::{PA04, PA10, PA11},
        Output, Pin, PushPull,
    },
    prelude::*,
};

pub struct Stepper {
    enable: Pin<PA04, Output<PushPull>>,
    direction: Pin<PA10, Output<PushPull>>,
    step: Pin<PA11, Output<PushPull>>,
}

impl Stepper {
    pub fn init(
        enable: Pin<PA04, Output<PushPull>>,
        direction: Pin<PA10, Output<PushPull>>,
        step: Pin<PA11, Output<PushPull>>,
    ) -> Self {
        let mut motor = Self {
            enable,
            direction,
            step,
        };

        motor.disable();
        motor.cw();
        motor
    }

    pub fn enable(&mut self) {
        self.enable.set_low();
    }
    pub fn disable(&mut self) {
        self.enable.set_high();
    }
    pub fn cw(&mut self) {
        self.direction.set_high();
    }
    pub fn ccw(&mut self) {
        self.direction.set_low();
    }
    pub fn do_step(&mut self) {
        self.step.toggle();
    }
}
