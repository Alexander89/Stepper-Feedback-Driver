#![no_main]
#![cfg_attr(not(test), no_std)]
#![allow(deprecated)]

mod devices;

use devices::Devices;
use hal::pac::{self, interrupt};

use utils::time::U32Ext;
use xiao_m0::{entry, hal};

static mut HARDWARE: Option<Devices> = None;

#[entry]
fn main() -> ! {
    run()
}
fn run() -> ! {
    let hardware = unsafe {
        cortex_m::interrupt::free(|cs| {
            HARDWARE = Some(Devices::init(cs));
            HARDWARE.as_mut().unwrap()
        })
    };

    init(hardware);
    hardware.stepper_enable();
    hardware.delay_us(100_000.us());

    //let execute_stepper_time_consumptions_us = 190.us(); // measured in debugger :-(
    let execute_stepper_time_consumptions_us = 50.us(); // measured in debugger :-(

    // in us
    let dt_min: f32 = 1_000_000.0 / 200.0; // 200 steps per sec
    let dt_max: f32 = 1_000_000.0 / 1250.0; // 950 steps per sec

    let slope_delta_t: f32 = 1_500_000f32; // us to ramp up  (1 sec)
    let slope: f32 = (dt_max - dt_min) as f32 / slope_delta_t;

    let f = |t: f32| slope * t + dt_min;
    let g = |dt: f32| (dt - dt_min) / slope;
    let h = |dt: f32| g(dt) + dt;
    let calc_dt_min = |dt: f32| f(h(dt.min(dt_min).max(dt_max))).max(dt_max);

    let mut sensor_poll_delay = 0.us();
    let sensor_poll_timeout = 1_000.us();

    let mut last_dt = 0.us(); // us / steps

    loop {
        sensor_poll_delay += hardware.get_delta_us_1();
        // poll I²C - magnet sensor.
        if sensor_poll_delay >= sensor_poll_timeout {
            // split magnet sensor write and read into two functions for two cycle runs
            hardware.stepwise_read();
            sensor_poll_delay = 0.us();
        }

        // poll motor for next step?
        match hardware.poll_stepper() {
            // nothing to do, just poll again and again and again
            devices::NextStepperAction::Idle => {}

            // hardware requires a little delay after changing the direction
            req @ devices::NextStepperAction::DirectionChanged(_) => {
                hardware.execute_stepper(req);
                // @WARNING - Program waits here // no polling in this time
                hardware.delay_us(150.us());

                // drop the speed to 0 steps/sec, to ramp up again.
                // 0xFFFF_FFFF ~~~ 1/0  // (1/V)
                last_dt = 100_000.us();
            }
            // if a step is required, check if we have to wait before we can do it
            req @ devices::NextStepperAction::StepRequired(_) => {
                // get delta-time to the last step.
                let dt = hardware.peek_delta_us_0();

                // check if we are slower than the last step
                // execute_stepper_time_consumptions_us add some µs the execute_stepper
                // would roughly take.
                if dt + execute_stepper_time_consumptions_us > last_dt {
                    hardware.execute_stepper(req);
                    // Get delta again to get the most accurate delta between the steps
                    last_dt = hardware.get_delta_us_0();
                } else {
                    // @todo why last_dt
                    let next_dt_min = (calc_dt_min(last_dt.0 as f32) as u32).us();

                    if dt + execute_stepper_time_consumptions_us < next_dt_min {
                        let delta_t_last_step = hardware.peek_delta_us_0();

                        if next_dt_min > delta_t_last_step {
                            let open_delay = next_dt_min - delta_t_last_step;

                            // HACK jump out to the beginning of the loop is not really nice here
                            if open_delay > 1200.us() {
                                continue;
                            }
                            // @WARNING - Program waits here
                            hardware.delay_us(open_delay);
                        }
                        hardware.execute_stepper(req);

                        // set last speed to calculated v_max value to avoid inaccuracy in calculation
                        hardware.get_delta_us_0();
                        // HACK us calculated delta T to pretend that the result is accurate
                        last_dt = next_dt_min;
                    } else {
                        if hardware.execute_stepper(req) {
                            last_dt = hardware.get_delta_us_0();
                        } else {
                            last_dt = 100.ms().into();
                        }
                    }
                }
            }
        }
    }
}

fn init(hw: &mut Devices) {
    hw.led0.on();
    hw.delay(100.ms());

    hw.poll_magnet_sensor_setup();

    hw.led0.off();
    hw.delay(400.ms());

    while hw.magnet_sensor.magnitude < 1000 {
        hw.led1.on();
        hw.led2.off();
        hw.delay(400.ms());
        hw.led1.off();
        hw.led2.on();
        hw.delay(400.ms());

        hw.poll_magnet_sensor_setup();
    }
    hw.led0.off();
    hw.led1.on();

    hw.poll_magnet_sensor();
    let steps = hw.get_step();
    hw.init_stepper(steps);

    hw.delay(400.ms());
    hw.led0.on();
    hw.led1.off();
    hw.execute_ext_int_pins();

    hw.delay(400.ms());
    hw.led0.off();
}

fn _debug_timer(_d_t: u32) {
    #[cfg(feature = "serial")]
    {
        unsafe { HARDWARE.as_mut() }.map(|hw| {
            let _ = hw.serial_write_num(hw.get_step() as usize);
            // let _ = hw.serial_write(b" ");
            // let _ = hw.serial_write_num(hw.magnet_sensor.magnitude as usize);
            let _ = hw.serial_write(b"\r\n");
        });
    }
}

fn dir_changed(state: bool) {
    unsafe { HARDWARE.as_mut() }.map(|hw| {
        if state {
            hw.stepper_cw();
            hw.led0.on()
        } else {
            hw.stepper_ccw();
            hw.led0.off()
        }
    });
}

fn enabled_changed(state: bool) {
    unsafe { HARDWARE.as_mut() }.map(|hw| {
        if state {
            hw.stepper_enable();
            hw.led1.on()
        } else {
            hw.stepper_disable();
            hw.led1.off()
        }
    });
}
fn step_changed(state: bool) {
    unsafe { HARDWARE.as_mut() }.map(|hw| {
        hw.stepper_step();
        if state {
            hw.led2.on()
        } else {
            hw.led2.off()
        }
    });
}

#[cfg(feature = "serial")]
#[interrupt]
fn USB() {
    let hwo = unsafe { HARDWARE.as_mut() };
    hwo.map(|hw| {
        hw.led2.on();
        hw.poll_serial();
        hw.serial_read_poll();
    });
}

#[interrupt]
fn EIC() {
    let hwo = unsafe { HARDWARE.as_mut() };
    hwo.map(|hw| {
        hw.handle_eic();
    });
}

#[interrupt]
fn RTC() {
    let hwo = unsafe { HARDWARE.as_mut() };
    hwo.map(|hw| {
        hw.handle_rtc_overflow();
    });
}
// #[interrupt]
// fn TC3() {
//     unsafe {
//         HARDWARE.as_mut().map(|hw| {
//             hw.led1.toggle();
//         });
//     };
// }

// #[interrupt]
// fn TC3() {
//     let p = unsafe { pac::TC3::ptr().as_ref() }.unwrap();
//     p.count16().intflag.modify(|_, w| w.ovf().set_bit());
//     step_timer(0);
// }

// #[interrupt]
// fn TC4() {
//     let p = unsafe { pac::TC4::ptr().as_ref() }.unwrap();
//     p.count16().intflag.modify(|_, w| w.ovf().set_bit());
//     step_timer(0);
// }

// #[interrupt]
// fn TC5() {
//     let p = unsafe { pac::TC5::ptr().as_ref() }.unwrap();
//     p.count16().intflag.modify(|_, w| w.ovf().set_bit());
//     step_timer(0);
// }

#[cfg(not(test))]
#[inline(never)]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    use cortex_m::asm::nop;
    unsafe {
        let hw = HARDWARE.as_mut().unwrap();

        loop {
            for _ in 0..0xffff {
                nop();
            }
            hw.led0.off();
            hw.led1.off();

            for _ in 0..0xffff {
                nop();
            }
            hw.led0.on();
            hw.led1.on();
        }
    }
}
