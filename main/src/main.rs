#![no_main]
#![cfg_attr(not(test), no_std)]
#![allow(deprecated)]

mod devices;

use core::panic::PanicInfo;
use devices::Devices;
use hal::{pac::interrupt, prelude::*};

use xiao_m0::{entry, hal};

static mut HARDWARE: Option<Devices> = None;

#[entry]
fn main() -> ! {
    let hardware = unsafe {
        HARDWARE = Some(Devices::init());
        HARDWARE.as_mut().unwrap()
    };

    hardware.delay(3000.ms());
    loop {
        hardware.stepper_enable();
        for _ in 0..50 {
            hardware.delay(100.ms());
            hardware.stepper_step();
        }

        hardware.stepper_disable();
        hardware.delay(3000.ms());
    }
}

#[interrupt]
fn USB() {
    poll_usb();
}

fn poll_usb() {
    unsafe {
        HARDWARE.as_mut().map(|hw| {
            hw.poll_serial();
        });
    };
}

#[cfg(not(test))]
#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    unsafe {
        HARDWARE.as_mut().map(|hw| {
            hw.led0.off();
            hw.led1.off();
        });
    };

    loop {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}
