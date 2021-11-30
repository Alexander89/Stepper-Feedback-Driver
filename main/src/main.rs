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

    hardware.delay_us(50_000.us());

    let mut last_dt = 0.us(); // us / steps
    let mut last_v = 0f32; // us / steps

    let execute_stepper_time_consumptions_us = 190.us(); // measured in debugger :-(
                                                         //let slope: f32 = 0.76;
    let slope: f32 = 100.0;

    // TODO change unit from sec to µs
    let v_min: f32 = 200.0;
    let v_max: f32 = 950.0;

    // TODO change from velocity to deltaT
    let f = |t: f32| (slope * t + v_min).min(v_max);
    let g = |v: f32| (v.min(v_max) - v_min) / slope;
    let s = |v: f32| 1.0 / v;
    let h = |v: f32| g(v) + s(v);
    let f_v_max = |v: f32| f(h(v.max(v_min)));

    let sensor_poll_timeout = 15_000.us();
    let mut sensor_poll_dt = 0.us();
    let mut last_poll = 0.us();

    loop {
        let dt = hardware.peek_delta_us();
        // use sensor_poll_dt to make the delay independent of the steps independent
        if (sensor_poll_dt + (dt - last_poll)) >= sensor_poll_timeout {
            hardware.poll_magnet_sensor();
            last_poll = dt;
            sensor_poll_dt = 0.us();
        }

        let _t_02 = hardware.peek_delta_us() - dt;

        match hardware.poll_stepper() {
            // nothing to do, just poll again and again and again
            devices::StepPollResult::Idle => {}

            // hardware requires a little delay after changing the direction
            devices::StepPollResult::DirectionChanged => {
                let _t_0_01 = hardware.peek_delta_us() - dt;
                // @WARNING - Program waits here
                hardware.delay_us(150.us());
                // drop the speed to 0 steps/sec, to ramp up again.
                // 0xFFFF_FFFF ~~~ 1/0  // (1/V)
                last_dt = 100.ms().into();
                last_v = 0f32;
            }

            // if a step is required, check if we have to wait before we can do it
            req @ devices::StepPollResult::StepRequired(_) => {
                let _t_1_01 = hardware.peek_delta_us() - dt;
                // get delta-time to the last step.
                let dt = hardware.get_delta_us();
                let _t_1_02 = hardware.peek_delta_us();

                // check if we are slower than the last step
                // execute_stepper_time_consumptions_us add some µs the execute_stepper
                // would roughly take.
                if dt + execute_stepper_time_consumptions_us > last_dt {
                    hardware.execute_stepper(req);
                    // Get delta again to get the most accurate delta between the steps
                    last_dt = dt + hardware.get_delta_us();
                    last_v = 1_000_000f32 / dt.0 as f32;
                } else {
                    // v [steps / Sec]
                    let _t_1_03 = hardware.peek_delta_us();

                    let v: f32 = (1_000_000f32 / dt.0 as f32).max(v_min);
                    let _t_1_04 = hardware.peek_delta_us();
                    let v_max = f_v_max(last_v as f32);
                    let _t_1_05 = hardware.peek_delta_us();

                    if v > v_max {
                        let min_delay = (((1f32 / v_max) * 1_000_000f32) as u32).us()
                            - execute_stepper_time_consumptions_us;
                        let delta_t_last_step = dt + hardware.get_delta_us();

                        if min_delay > delta_t_last_step {
                            let open_delay = min_delay - delta_t_last_step;

                            // @WARNING - Program waits here
                            hardware.delay_us(open_delay);
                        }

                        hardware.execute_stepper(req);

                        last_dt = delta_t_last_step + hardware.get_delta_us();
                        // set last speed to calculated v_max value to avoid inaccuracy in calculation
                        last_v = v_max;
                    } else {
                        if hardware.execute_stepper(req) {
                            last_v = v_max;
                        } else {
                            last_dt = 100.ms().into();
                            last_v = 0f32;
                        }
                    }
                }

                // add last_dt to the sensor_poll_dt to measure time independent of the steps
                sensor_poll_dt = sensor_poll_dt + last_dt;
                last_poll = 0.us();
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
        let file = _info.location().unwrap().file().as_bytes();

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
