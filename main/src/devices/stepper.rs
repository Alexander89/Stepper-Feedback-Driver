use atsamd_hal::{
    delay::Delay,
    gpio::v2::{Pin, PushPullOutput, PA04, PA10, PA11},
    prelude::*,
};
use embedded_hal::digital::v2::PinState;

pub enum Direction {
    CW,
    CCW,
}

pub enum NextStepperAction {
    Idle,
    DirectionChanged(Direction),
    StepRequired(i8),
}
pub struct Stepper {
    enable: Pin<PA04, PushPullOutput>,
    direction: Pin<PA10, PushPullOutput>,
    step: Pin<PA11, PushPullOutput>,
    state: PinState,
    turn_cw: bool,
    current_direction_cw: bool,
    target_step: i32,
    current_step: i32,
    real_step: i32,
    last_mag_val: i16,
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
            target_step: 0,
            current_step: 0,
            real_step: 0,
            last_mag_val: 0,
        };
        motor.direction.set_high();

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
            self.target_step += 100;
        } else {
            self.target_step -= 100;
        }
    }

    pub fn init_stepper(&mut self, start_value: i16) {
        self.last_mag_val = start_value;
    }

    pub fn poll_next_action(&mut self) -> NextStepperAction {
        if self.current_step == self.target_step {
            // nop / rest
            NextStepperAction::Idle
        } else if self.current_step > self.target_step && self.current_direction_cw {
            // change dir to CCW
            NextStepperAction::DirectionChanged(Direction::CCW)
        } else if self.current_step < self.target_step && !self.current_direction_cw {
            // change dir to CW
            NextStepperAction::DirectionChanged(Direction::CW)
        } else {
            // do step now
            if self.current_direction_cw {
                NextStepperAction::StepRequired(1)
            } else {
                NextStepperAction::StepRequired(-1)
            }
        }
    }

    pub fn execute(&mut self, req: NextStepperAction) -> bool {
        match req {
            NextStepperAction::Idle => false,
            NextStepperAction::DirectionChanged(Direction::CW) => {
                self.direction.set_high();
                self.current_direction_cw = true;
                true
            }
            NextStepperAction::DirectionChanged(Direction::CCW) => {
                self.direction.set_low();
                self.current_direction_cw = false;
                true
            }
            NextStepperAction::StepRequired(i) => {
                // do step now
                self.current_step += i as i32;

                self.state = !self.state;
                self.step.set_high();
                cortex_m::asm::nop();
                cortex_m::asm::nop();
                cortex_m::asm::nop();
                cortex_m::asm::nop();
                cortex_m::asm::nop();
                self.step.set_low();

                // HACK do a better stuck protection
                (self.current_step - self.real_step).abs() < 3
            }
        }
    }

    pub fn update_angle(&mut self, mag_val: i16) {
        // 1. normalize: move last to 100
        let mmove = 100 - self.last_mag_val;
        let norm_current = self.last_mag_val + mmove;
        // 2.  move new val too
        let mut new_val = mag_val + mmove;
        // 3. fix overflow
        if new_val < 0 {
            new_val += 200;
        } else if new_val > 200 {
            new_val -= 200;
        }

        // get movement
        let mut dif: i32 = (norm_current - new_val) as i32;

        // fix real step value
        self.real_step -= dif;

        // TODO fix reading - adjust to sensor readings
        if self.real_step < self.current_step - 1 {
            self.current_step -= (self.current_step - self.real_step) / 2;
        } else if self.real_step > self.current_step + 1 {
            self.current_step += (self.real_step - self.current_step) / 2;
        }

        self.last_mag_val = mag_val;
    }
}
