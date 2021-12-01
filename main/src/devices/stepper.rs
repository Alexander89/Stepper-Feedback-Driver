use atsamd_hal::{
    delay::Delay,
    gpio::v2::{Pin, PushPullOutput, PA04, PA10, PA11},
    prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin,
};
use embedded_hal::digital::v2::PinState;
use utils::time::{Microseconds, U32Ext};

use crate::settings::{DT_MIN_U32, STEPS_PER_RESOLUTION_I16, STEPS_PER_RESOLUTION_I16_HALF};

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
    filtered_dt_per_step: u32,
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
            filtered_dt_per_step: 0,
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

    pub fn update_angle(
        &mut self,
        mag_val: i16,
        dt: Microseconds,
        motor_dt: Microseconds,
    ) -> Option<Microseconds> {
        // 1. normalize: move last to 100
        let mmove = STEPS_PER_RESOLUTION_I16_HALF - self.last_mag_val;

        // 2.  move new val too
        let mut new_val = mag_val + mmove;
        // 3. fix overflow
        if new_val < 0 {
            new_val += STEPS_PER_RESOLUTION_I16;
        } else if new_val > STEPS_PER_RESOLUTION_I16 {
            new_val -= STEPS_PER_RESOLUTION_I16;
        }

        // get movement
        let mut dif: i32 = (STEPS_PER_RESOLUTION_I16_HALF - new_val) as i32;

        // fix real step value
        self.real_step -= dif;

        // stuck protection -----------------------
        let dt_per_step = if dif == 0 {
            DT_MIN_U32
        } else {
            (dt.0 / dif.abs() as u32).min(DT_MIN_U32)
        };

        self.filtered_dt_per_step = ((self.filtered_dt_per_step + dt_per_step) / 2).min(DT_MIN_U32);
        let stuck = motor_dt.0 < self.filtered_dt_per_step - 1280; // - a_number_to_get_rid_of_time_inaccuracy ;

        // TODO fix reading - adjust to sensor readings
        let dif = self.current_step - self.real_step;
        match dif.abs() {
            1 => {
                // TODO tune readings first
                // this ends up in back and forward
                // self.current_step -= dif
            }
            x if x > 1 => self.current_step -= (dif) / 2,
            _ => (),
        }

        self.last_mag_val = mag_val;

        if stuck {
            Some(self.filtered_dt_per_step.us())
        } else {
            None
        }
    }
}
