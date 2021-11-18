use atsamd_hal::{
    delay::Delay,
    gpio::v2::{Pin, PushPullOutput, PA04, PA10, PA11},
    prelude::*,
};
use embedded_hal::digital::v2::PinState;

pub struct Stepper {
    enable: Pin<PA04, PushPullOutput>,
    direction: Pin<PA10, PushPullOutput>,
    step: Pin<PA11, PushPullOutput>,
    state: PinState,
    turn_cw: bool,
    current_direction_cw: bool,
    target_step: i32,
    current_step: i32,
}

impl Stepper {
    pub fn init(
        enable: Pin<PA04, PushPullOutput>,
        direction: Pin<PA10, PushPullOutput>,
        step: Pin<PA11, PushPullOutput>,
    ) -> Self {
        let mut motor = Self {
            enable,
            direction,
            step,
            state: PinState::Low,
            turn_cw: true,
            current_direction_cw: true,
            target_step: 0i32,
            current_step: 0i32,
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
        self.turn_cw = true;
    }
    pub fn ccw(&mut self) {
        self.turn_cw = false;
    }
    pub fn do_step(&mut self) {
        if self.turn_cw {
            self.target_step += 1;
        } else {
            self.target_step -= 1;
        }
    }

    pub fn init_stepper(&mut self, start_value: i32) {
        self.current_step = start_value;
    }

    pub fn poll(&mut self) {
        if (self.current_step == self.target_step) {
            // nop / rest
        } else if self.current_step > self.target_step && self.current_direction_cw {
            self.direction.set_low();
            self.current_direction_cw = false;
        } else if self.current_step < self.target_step && !self.current_direction_cw {
            self.direction.set_high();
            self.current_direction_cw = true;
        } else {
            if self.current_direction_cw {
                self.current_step += 1;
            } else {
                self.current_step -= 1;
            }

            self.state = !self.state;
            self.step.set_state(self.state);
        }
    }
}
