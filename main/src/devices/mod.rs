#![allow(unused)]
mod ext_int_pin;
mod i2c;
mod led;
mod magnet_sensor;
mod stepper;
mod tc32;
mod usb_serial;

use atsamd_hal::{
    clock::{ClockGenId, ClockSource},
    gpio::v2::{PA08, PB09},
    rtc::{self, Count32Mode, Rtc},
    timer::TimerCounter,
    timer_traits::InterruptDrivenTimer,
};
use cortex_m::{asm::nop, interrupt::CriticalSection};
use embedded_hal::blocking::delay::*;
use usb_device::UsbError;
use utils::{
    num_to_string,
    time::{Microseconds, Milliseconds, U32Ext},
};
use xiao_m0::hal::{
    clock::GenericClockController,
    delay::Delay,
    eic::pin::ExtInt1,
    gpio::v2::{PA02, PA05, PA06, PA07, PA09, PA17, PA18, PA19},
    pac::{self, interrupt, CorePeripherals, Peripherals},
    sercom::{v2::uart::Clock, I2CError},
    target_device::{tc3::COUNT32, NVIC, RTC, TC3, TC4, TC5},
    timer,
};

use ext_int_pin::ExtIntPin;
use i2c::I2c;
pub use stepper::StepPollResult;
use stepper::Stepper;
use usb_serial::UsbSerial;

use self::{
    led::Led,
    magnet_sensor::MagnetSensor,
    tc32::{Count32, TimerCounter32},
};

pub struct Devices {
    clocks: GenericClockController,
    i2c: I2c,
    #[cfg(feature = "serial")]
    serial: UsbSerial,
    stepper: Stepper,
    pub magnet_sensor: MagnetSensor,

    delay: Delay,

    pub rtc: Rtc<Count32Mode>,
    rtc_ovl: bool,

    pub led0: Led<PA17>,
    pub led1: Led<PA18>,
    pub led2: Led<PA19>,

    dir: ExtIntPin<PB09>,    // pin 7
    step: ExtIntPin<PA05>,   // pin 9
    enable: ExtIntPin<PA06>, // pin 10

    timer_0_buffer: u32,
    timer_1_buffer: u32,
}

impl Devices {
    pub fn get_step(&self) -> i16 {
        let r = (self.magnet_sensor.raw_angle as i32 * 50 + 512) / 1024;
        if r >= 200 {
            (r - 200) as i16
        } else {
            r as i16
        }
    }

    #[inline]
    pub fn get_delta_us_0(&mut self) -> Microseconds {
        if self.rtc_ovl {
            self.rtc_ovl = false;
            // hmmmmmm
            self.timer_1_buffer = 1_000_000;
            1_000_000.us()
        } else {
            // no swap possible
            let v = self.rtc.count32();
            self.rtc.set_count32(0);

            self.timer_1_buffer += v;
            let l = self.timer_0_buffer + v;
            self.timer_0_buffer = 0;
            l.us()
        }
    }

    #[inline]
    pub fn get_delta_us_1(&mut self) -> Microseconds {
        if self.rtc_ovl {
            self.rtc_ovl = false;
            // hmmmmmm
            self.timer_0_buffer = 1_000_000;
            1_000_000.us()
        } else {
            // no swap possible
            let v = self.rtc.count32();
            self.rtc.set_count32(0);

            self.timer_0_buffer += v;
            let l = self.timer_1_buffer + v;
            self.timer_1_buffer = 0;
            l.us()
        }
    }
    #[inline]
    pub fn peek_delta_us_0(&mut self) -> Microseconds {
        if self.rtc_ovl {
            1_000_000.us()
        } else {
            (self.timer_0_buffer + self.rtc.count32()).us()
        }
    }
    #[inline]
    pub fn peek_delta_us_1(&mut self) -> Microseconds {
        if self.rtc_ovl {
            1_000_000.us()
        } else {
            (self.timer_1_buffer + self.rtc.count32()).us()
        }
    }

    pub fn handle_rtc_overflow(&mut self) {
        self.rtc_ovl = true;
        self.rtc.set_count32(0);
    }

    pub fn handle_eic(&mut self) {
        let eic = unsafe { &*pac::EIC::ptr() };
        while eic.status.read().syncbusy().bit_is_set() {}

        self.dir.poll();
        self.enable.poll();
        self.step.poll();
    }
    pub fn execute_ext_int_pins(&mut self) {
        self.dir.execute();
        self.enable.execute();
        self.step.execute();
    }

    pub fn query_magnet_sensor(&mut self) {
        self.magnet_sensor.query(&mut self.i2c);
    }

    pub fn read_magnet_sensor_result(&mut self) {
        self.magnet_sensor.read(&mut self.i2c);
        if self.magnet_sensor.detected {
            self.stepper.update_angle(self.get_step());
        }
    }

    pub fn poll_magnet_sensor(&mut self) {
        self.query_magnet_sensor();
        self.read_magnet_sensor_result()
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
    pub fn poll_serial(&mut self) {
        #[cfg(feature = "serial")]
        self.serial.poll()
    }
    pub fn serial_read_poll(&mut self) {
        #[cfg(feature = "serial")]
        {
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
    }

    pub fn serial_write(&mut self, bytes: &[u8]) {
        #[cfg(feature = "serial")]
        self.serial_write_len(&bytes, bytes.len())
    }

    pub fn serial_write_num(&mut self, num: usize) {
        #[cfg(feature = "serial")]
        {
            let (len, bytes) = num_to_string(num);
            self.serial_write_len(&bytes, len)
        }
    }

    pub fn serial_write_len(&mut self, bytes: &[u8], len: usize) {
        #[cfg(feature = "serial")]
        {
            self.serial.serial_write_len(bytes, len);
            self.delay_us((15 * (len as u32)).us());
            self.poll_serial();
        }
    }
}

impl Devices {
    pub fn init_stepper(&mut self, start_value: i16) {
        self.stepper.init_stepper(start_value)
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
    pub fn poll_stepper(&mut self) -> stepper::StepPollResult {
        self.stepper.poll()
    }
    pub fn execute_stepper(&mut self, req: stepper::StepPollResult) -> bool {
        self.stepper.execute(req)
    }
}

impl Devices {
    pub fn init(cs: &CriticalSection) -> Self {
        let mut peripherals = Peripherals::take().unwrap();
        let mut core = CorePeripherals::take().unwrap();
        let mut clocks = GenericClockController::with_internal_8mhz(
            peripherals.GCLK,
            &mut peripherals.PM,
            &mut peripherals.SYSCTRL,
            &mut peripherals.NVMCTRL,
        );

        let mut pins = xiao_m0::Pins::new(peripherals.PORT);
        let mut led0 = Led::init(pins.led0.into_push_pull_output());
        let mut led1 = Led::init(pins.led1.into_push_pull_output());
        let mut led2 = Led::init(pins.led2.into_push_pull_output());

        let mut delay = Delay::new(core.SYST, &mut clocks);

        #[cfg(feature = "serial")]
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

        blink(1, &mut led0, &mut led1, &mut led2, &mut delay);

        ext_int_pin::init(
            &cs,
            &mut clocks,
            &mut core.NVIC,
            &mut peripherals.PM,
            peripherals.EIC,
        );
        let enable = ExtIntPin::<PA06>::enable(pins.a10, super::enabled_changed);
        let step = ExtIntPin::<PA05>::enable(pins.a9, super::step_changed);
        let dir = ExtIntPin::<PB09>::enable(pins.a7, super::dir_changed);

        let gclk3 = clocks
            .configure_gclk_divider_and_source(
                ClockGenId::GCLK3,
                1,
                pac::gclk::genctrl::SRC_A::OSC8M,
                false,
            )
            .unwrap();
        let rtc_clock_gen = &clocks.rtc(&gclk3).unwrap();
        let mut rtc = Rtc::count32_mode(peripherals.RTC, rtc_clock_gen.freq(), &mut peripherals.PM);
        rtc.reset_and_compute_prescaler::<atsamd_hal::time::Milliseconds>(64.ms().into());
        rtc.enable_interrupt();

        unsafe {
            // core.NVIC.set_priority(interrupt::RTC, 2);
            // NVIC::unmask(interrupt::RTC);
        }
        blink(2, &mut led0, &mut led1, &mut led2, &mut delay);

        Self {
            clocks,

            i2c,
            #[cfg(feature = "serial")]
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

            rtc,
            rtc_ovl: false,

            timer_0_buffer: 0,
            timer_1_buffer: 0,
        }
    }
}

fn blink<A, B, C>(
    count: u8,
    led0: &mut Led<A>,
    led1: &mut Led<B>,
    led2: &mut Led<C>,
    delay: &mut Delay,
) where
    A: atsamd_hal::gpio::PinId,
    B: atsamd_hal::gpio::PinId,
    C: atsamd_hal::gpio::PinId,
{
    for i in 0..count {
        led0.on();
        led1.on();
        led2.on();
        delay.delay_ms(200u16);

        led0.off();
        led1.off();
        led2.off();
        delay.delay_ms(200u16);
    }
    delay.delay_ms(800u16);
}
