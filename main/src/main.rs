#![no_main]
#![cfg_attr(not(test), no_std)]
#![allow(deprecated)]

mod devices;

use core::panic::PanicInfo;
use devices::Devices;
use hal::{
    gpio::{
        v2::{Alternate, OutputConfig, D, PA08, PA09},
        Output, Pin, PinId,
    },
    pac::interrupt,
    prelude::*,
    sercom::{
        v2::{Pad0, Pad1},
        I2CError, I2CMaster2, Pad,
    },
    target_device::SERCOM2,
};

use xiao_m0::{entry, hal};

static mut HARDWARE: Option<Devices> = None;

static mut I2C: Option<
    I2CMaster2<
        Pad<SERCOM2, Pad0, Pin<PA08, Alternate<D>>>,
        Pad<SERCOM2, Pad1, Pin<PA09, Alternate<D>>>,
    >,
> = None;

#[entry]
fn main() -> ! {
    let hardware = unsafe {
        HARDWARE = Some(Devices::init());
        HARDWARE.as_mut().unwrap()
    };

    hardware.delay(3000.ms());

    loop {}
}

// fn read_i2c<A, P0, P1>(address: A, start_reg: u8)
// where
//     A: AddressMode,
//     P0: CompatiblePad<Sercom = SERCOM2, PadNum = Pad0>,
//     P1: CompatiblePad<Sercom = SERCOM2, PadNum = Pad1>,
// {
// }

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
fn panic(info: &PanicInfo) -> ! {
    HARDWARE.as_mut().map(|hw| {
        hw.led0.off();
        hw.led1.off();
    });

    loop {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }
}
