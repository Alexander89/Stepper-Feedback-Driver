#![allow(dead_code)]

use atsamd_hal::{
    clock::GenericClockController,
    gpio::v2::{AnyPin, PA24, PA25},
    target_device::NVIC,
};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
    UsbError,
};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use utils::num_to_string;
use xiao_m0::{
    pac,
    pac::{interrupt, CorePeripherals},
    UsbBus,
};

static mut BUS_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;

pub struct UsbSerial {
    bus: UsbDevice<'static, UsbBus>,
    serial: SerialPort<'static, UsbBus>,
}

impl UsbSerial {
    pub fn poll(&mut self) {
        self.bus.poll(&mut [&mut self.serial]);
    }
}

impl UsbSerial {
    pub fn init(
        clocks: &mut GenericClockController,
        usb: pac::USB,
        pm: &mut pac::PM,
        dm: impl AnyPin<Id = PA24>,
        dp: impl AnyPin<Id = PA25>,
        core: &mut CorePeripherals,
    ) -> UsbSerial {
        let bus_allocator = unsafe {
            BUS_ALLOCATOR = Some(xiao_m0::usb_allocator(
                usb, clocks, pm, //&mut peripherals.PM,
                dm, //pins.usb_dm,
                dp, // pins.usb_dp,
            ));
            BUS_ALLOCATOR.as_mut().unwrap()
        };

        let serial = SerialPort::new(bus_allocator);
        let bus = UsbDeviceBuilder::new(bus_allocator, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Halemba")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();
        unsafe {
            core.NVIC.set_priority(interrupt::USB, 1);
            NVIC::unmask(interrupt::USB);
        };
        UsbSerial { bus, serial }
    }

    pub fn serial_write(&mut self, bytes: &[u8]) -> Result<(), UsbError> {
        self.serial_write_len(&bytes, bytes.len())
    }

    pub fn serial_write_num(&mut self, num: usize) -> Result<(), UsbError> {
        let (len, bytes) = num_to_string(num);
        self.serial_write_len(&bytes, len)
    }

    pub fn serial_write_len(&mut self, bytes: &[u8], len: usize) -> Result<(), UsbError> {
        self.serial.write(&bytes[0..len])?;
        self.serial.flush()
    }
}
