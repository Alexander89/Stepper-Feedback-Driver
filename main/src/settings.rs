use utils::time::Microseconds;

pub const STEPS_PER_RESOLUTION_I16: i16 = 200;
pub const STEPS_PER_RESOLUTION_I16_HALF: i16 = 100;
pub const STEPS_PER_RESOLUTION_I16_QUARTER: i32 = 50;

pub const EXECUTE_STEPPER_TIME_CONSUMPTIONS_US: Microseconds = Microseconds(50); // measured in debugger :-(

pub const SLOPE_DELTA_T: f32 = 1_500_000f32; // us to ramp up  (1 sec)
pub const SLOPE: f32 = (DT_MAX - DT_MIN) as f32 / SLOPE_DELTA_T;
pub const DT_MIN: f32 = 1_000_000.0 / 200.0; // 200 steps per sec
pub const DT_MIN_U32: u32 = DT_MIN as u32;
pub const DT_MAX: f32 = 1_000_000.0 / 1250.0; // 950 steps per sec
pub const DT_MAX_U32: u32 = DT_MAX as u32;
