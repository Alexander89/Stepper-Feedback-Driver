#![allow(unused)]
mod ext_int_pin;
mod i2c;
mod led;
mod magnet_sensor;
mod stepper;
mod usb_serial;

use atsamd_hal::{
    clock::ClockGenId,
    gpio::v2::{PA08, PB09},
    timer::TimerCounter,
};
use cortex_m::asm::nop;
use usb_device::UsbError;
use utils::num_to_string;
use xiao_m0::hal::{
    clock::GenericClockController,
    delay::Delay,
    eic::pin::ExtInt1,
    gpio::v2::{PA02, PA05, PA06, PA07, PA09, PA17, PA18, PA19},
    pac::{self, interrupt, CorePeripherals, Peripherals},
    prelude::*,
    sercom::{v2::uart::Clock, I2CError},
    target_device::{tc3::COUNT32, NVIC, RTC, TC3, TC4, TC5},
    time::{Microseconds, Milliseconds},
    timer,
};

use ext_int_pin::ExtIntPin;
use i2c::I2c;
use stepper::Stepper;
use usb_serial::UsbSerial;

use self::{led::Led, magnet_sensor::MagnetSensor};

pub struct Devices {
    clocks: GenericClockController,
    i2c: I2c,
    serial: UsbSerial,
    stepper: Stepper,
    pub magnet_sensor: MagnetSensor,

    delay: Delay,
    // pub timer0: timer::TimerCounter3,
    // pub timer3: timer::TimerCounter3,
    // pub timer4: timer::TimerCounter4,
    pub led0: Led<PA17>,
    pub led1: Led<PA18>,
    pub led2: Led<PA19>,

    // pub rtc: atsamd_hal::rtc::Rtc<atsamd_hal::rtc::Count32Mode>,
    dir: ExtIntPin<PB09>,    // pin 7
    step: ExtIntPin<PA05>,   // pin 9
    enable: ExtIntPin<PA06>, // pin 10
}

impl Devices {
    pub fn get_step(&self) -> u32 {
        let r = (self.magnet_sensor.raw_angle as u32 * 50 + 512) / 1024;
        if r >= 200 {
            r - 200
        } else {
            r
        }
    }

    pub fn poll_serial(&mut self) {
        self.serial.poll();
    }
    pub fn serial_read_poll(&mut self) {
        let res = self.serial.read_poll();
        if let Ok((size, buf)) = res {
            if size > 1 && buf[0] == b'd' {
                self.stepper.disable()
            } else if size > 1 && buf[0] == b'e' {
                self.stepper.enable()
            }
            self.led2.toggle();
        }
    }

    fn handle_input(&mut self, buf: &[u8; 100], size: usize) {
        self.stepper_disable();
    }

    pub fn handle_eic(&mut self) {
        nop();
        nop();
        nop();
        nop();

        self.dir.poll();
        self.enable.poll();
        self.step.poll();
    }

    pub fn poll_magnet_sensor(&mut self) {
        self.magnet_sensor.poll(&mut self.i2c);
    }
    pub fn poll_magnet_sensor_setup(&mut self) {
        self.magnet_sensor.poll_setup(&mut self.i2c);
    }

    pub fn delay(&mut self, time: Milliseconds) {
        self.delay.delay_ms(time.0);
    }

    pub fn delay_us(&mut self, time: Microseconds) {
        self.delay.delay_us(time.0);
    }

    pub fn stepper_step(&mut self) {
        self.stepper.do_step();
    }
    pub fn stepper_enable(&mut self) {
        self.stepper.enable();
    }
    pub fn stepper_disable(&mut self) {
        self.stepper.disable();
    }
    pub fn stepper_cw(&mut self) {
        self.stepper.cw();
    }
    pub fn stepper_ccw(&mut self) {
        self.stepper.ccw();
    }
    pub fn poll_stepper(&mut self) {
        self.stepper.poll();
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

    pub fn serial_write(&mut self, bytes: &[u8]) -> () {
        self.serial_write_len(&bytes, bytes.len())
    }

    pub fn serial_write_num(&mut self, num: usize) -> () {
        let (len, bytes) = num_to_string(num);
        self.serial_write_len(&bytes, len)
    }

    pub fn serial_write_len(&mut self, bytes: &[u8], len: usize) -> () {
        self.serial.serial_write_len(bytes, len);
        self.delay_us((15 * (len as u32)).us());
        self.poll_serial();
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
            &mut core.NVIC,
        );

        let i2c = I2c::init(
            &mut clocks,
            peripherals.SERCOM0,
            &mut peripherals.PM,
            pins.a4,
            pins.a5,
        );

        let stepper = Stepper::init(
            pins.a1.into_push_pull_output(),
            pins.a2.into_push_pull_output(),
            pins.a3.into_push_pull_output(),
        );

        let magnet_sensor = MagnetSensor::init();

        let led0 = Led::init(pins.led0.into_push_pull_output());
        let led1 = Led::init(pins.led1.into_push_pull_output());
        let led2 = Led::init(pins.led2.into_push_pull_output());

        let delay = Delay::new(core.SYST, &mut clocks);

        ExtIntPin::init(
            &mut clocks,
            &mut core.NVIC,
            &mut peripherals.PM,
            peripherals.EIC,
        );
        let enable = ExtIntPin::<PA06>::enable(pins.a10, super::enabled_changed);
        let step = ExtIntPin::<PA05>::enable(pins.a9, super::step_changed);
        let dir = ExtIntPin::<PB09>::enable(pins.a7, super::dir_changed);

        // let generator = {
        //     clocks.get_gclk(ClockGenId::GCLK2).unwrap_or_else(|| {
        //         clocks
        //             .configure_gclk_divider_and_source(
        //                 ClockGenId::GCLK2,
        //                 1,
        //                 pac::gclk::genctrl::SRC_A::OSC32K,
        //                 false,
        //             )
        //             .unwrap()
        //     })
        // };

        // // configure a clock for the TC4 and TC5 peripherals
        // let tc23_clock_gen = &clocks.tcc2_tc3(&generator).unwrap();
        // // instantiate a timer object for the TC4 peripheral
        // let mut timer3 = TimerCounter::tc3_(tc23_clock_gen, peripherals.TC3, &mut peripherals.PM);
        // // start a 5Hz timer
        // timer3.start(50.hz());
        // // enable interrupt
        // timer3.enable_interrupt();

        // // Enable TC3 interrupt in the NVIC
        // unsafe {
        //     core.NVIC.set_priority(interrupt::TC3, 2);
        //     NVIC::unmask(interrupt::TC3);
        // }

        // let tc45_clock_gen = &clocks.tc4_tc5(&generator).unwrap();
        // // instantiate a timer object for the TC6 peripheral
        // let mut timer4 = TimerCounter::tc4_(tc45_clock_gen, peripherals.TC4, &mut peripherals.PM);
        // // start a 5Hz timer
        // timer4.start(10.hz());
        // // enable interrupt
        // timer4.enable_interrupt();

        // Enable TC4 interrupt in the NVIC
        // unsafe {
        //     core.NVIC.set_priority(interrupt::TC4, 2);
        //     NVIC::unmask(interrupt::TC4);
        // }

        Self {
            clocks,

            i2c,
            serial,
            stepper,
            magnet_sensor,

            led0,
            led1,
            led2,

            delay,

            enable,
            step,
            dir,
            // timer3,
        }
    }
}
