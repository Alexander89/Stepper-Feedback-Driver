#![no_main]
#![cfg_attr(not(test), no_std)]
#![allow(deprecated)]

mod devices;

use devices::Devices;
use hal::{
    pac::{self, interrupt},
    prelude::*,
};

use xiao_m0::{entry, hal};

static mut HARDWARE: Option<Devices> = None;

#[entry]
fn main() -> ! {
    let hardware = unsafe {
        HARDWARE = Some(Devices::init());
        HARDWARE.as_mut().unwrap()
    };
    hardware.led0.on();

    hardware.stepper_enable();

    loop {
        hardware.poll_magnet_sensor();
        hardware.poll_magnet_sensor_setup();
        hardware.poll_stepper();
        hardware.delay_us(200.us());
    }
}

fn step_timer(_d_t: u32) {
    unsafe { HARDWARE.as_mut() }.map(|hw| {
        hw.led1.toggle();
    });
}

fn debug_timer(_d_t: u32) {
    unsafe { HARDWARE.as_mut() }.map(|hw| {
        let _ = hw.serial_write_num(hw.get_step() as usize);
        // let _ = hw.serial_write(b" ");
        // let _ = hw.serial_write_num(hw.magnet_sensor.magnitude as usize);
        let _ = hw.serial_write(b"\r\n");
    });
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

#[interrupt]
fn USB() {
    cortex_m::interrupt::free(|_| {
        unsafe {
            HARDWARE.as_mut().map(|hw| {
                hw.poll_serial();
                hw.serial_read_poll();
            });
        };
    })
}

#[interrupt]
fn EIC() {
    unsafe {
        HARDWARE.as_mut().map(|hw| {
            hw.handle_eic();
        });
    }
}

#[interrupt]
fn TC3() {
    unsafe {
        HARDWARE.as_mut().map(|hw| {
            hw.led1.toggle();
        });
    };
}
// #[interrupt]
// fn TC3() {
//     // Increase the counter and clear the interrupt.
//     let p = unsafe { pac::TC3::ptr().as_ref() }.unwrap();
//     p.count16().intflag.modify(|_, w| w.ovf().set_bit());
//     step_timer(0);
// }

// #[interrupt]
// fn TC4() {
//     // Increase the counter and clear the interrupt.
//     let p = unsafe { pac::TC4::ptr().as_ref() }.unwrap();
//     p.count16().intflag.modify(|_, w| w.ovf().set_bit());
//     // debug_timer(0);
// }

#[cfg(not(test))]
#[inline(never)]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    use cortex_m::asm::nop;

    let hw = unsafe { HARDWARE.as_mut() }.unwrap();

    loop {
        for _ in 0..0xfffff {
            nop();
        }
        hw.led0.off();
        hw.led1.off();

        for _ in 0..0xfffff {
            nop();
        }
        hw.led0.on();
        hw.led1.on();
    }
}
