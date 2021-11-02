#![allow(unused)]
mod i2c;
mod led;
mod stepper;
mod usb_serial;

use atsamd_hal::{prelude::*, sercom::I2CError, time::Milliseconds};
use xiao_m0::{
    clock::GenericClockController,
    delay::Delay,
    gpio::v2::{PA17, PA18},
    pac::{CorePeripherals, Peripherals},
};

use i2c::I2c;
use stepper::Stepper;
use usb_serial::UsbSerial;

use self::led::Led;

pub struct Devices {
    i2c: I2c,
    serial: UsbSerial,
    stepper: Stepper,

    delay: Delay,

    pub led0: Led<PA17>,
    pub led1: Led<PA18>,
}

impl Devices {
    pub fn poll_serial(&mut self) {
        self.serial.poll();
    }

    pub fn delay(&mut self, time: Milliseconds) {
        self.delay.delay_ms(time.0);
    }

    pub fn i2c_read_some(
        &mut self,
        address: u8,
        from: u8,
        count: usize,
        buffer: &mut [u8],
    ) -> Result<(), I2CError> {
        self.i2c.i2c_read_some(address, from, count, buffer)
    }
}

impl Devices {
    pub fn init() -> Self {
        let mut peripherals = Peripherals::take().unwrap();
        let mut core = CorePeripherals::take().unwrap();
        let mut clocks = GenericClockController::with_external_32kosc(
            peripherals.GCLK,
            &mut peripherals.PM,
            &mut peripherals.SYSCTRL,
            &mut peripherals.NVMCTRL,
        );
        let mut pins = xiao_m0::Pins::new(peripherals.PORT);

        let serial = UsbSerial::init(
            &mut clocks,
            peripherals.USB,
            &mut peripherals.PM,
            pins.usb_dm,
            pins.usb_dp,
            &mut core,
        );

        let i2c = I2c::init(
            &mut clocks,
            peripherals.SERCOM2,
            &mut peripherals.PM,
            pins.a4,
            pins.a5,
            &mut pins.port,
        );

        let stepper = Stepper::init(
            pins.a1.into_push_pull_output(&mut pins.port),
            pins.a2.into_push_pull_output(&mut pins.port),
            pins.a3.into_push_pull_output(&mut pins.port),
        );

        let led0 = Led::init(pins.led0.into_push_pull_output(&mut pins.port));
        let led1 = Led::init(pins.led1.into_push_pull_output(&mut pins.port));
        // ENA = Some(pins.a1.into_push_pull_output(&mut pins.port));
        // DIR = Some(pins.a2.into_push_pull_output(&mut pins.port));
        // STEP = Some(pins.a3.into_push_pull_output(&mut pins.port));

        let delay = Delay::new(core.SYST, &mut clocks);

        Self {
            i2c,
            serial,
            stepper,

            led0,
            led1,

            delay,
        }
    }
}
