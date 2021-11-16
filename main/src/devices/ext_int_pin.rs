use core::marker::PhantomData;
use core::sync::atomic::AtomicBool;
use core::{cell::RefCell, sync::atomic::Ordering};

use cortex_m::interrupt::{CriticalSection, Mutex};
use embedded_hal::digital::v2::PinState;
use hal::{
    clock::{ClockGenId, GenericClockController},
    eic::{
        pin::{self as extInt, ExternalInterrupt, Sense},
        EIC,
    },
    gpio::{
        v2::{
            self as pin_v2, AnyPin, DynPinId, Pin, PinId, PullDownInterrupt, PullUpInterrupt,
            PushPullOutput, PA01,
        },
        Output, Pa5, PinMode,
    },
    prelude::*,
    target_device::NVIC,
    target_device::{self, eic::RegisterBlock, CorePeripherals, Peripherals},
};
use xiao_m0::{
    hal,
    pac::{self, interrupt},
};

static EIC_SETUP: AtomicBool = AtomicBool::new(false);
static EIC: Mutex<RefCell<Option<EIC>>> = Mutex::new(RefCell::new(None));
pub struct ExtIntPin<I: PinId> {
    _p: PhantomData<I>,
    ext_id: u8,
    state: bool,
    cb: fn(bool) -> (),
}
impl ExtIntPin<PA01> {
    pub fn init(
        clocks: &mut GenericClockController,
        nvic: &mut NVIC,
        pm: &mut pac::PM,
        eic: pac::EIC,
    ) {
        // not supported by chip
        // let is_configured = EIC_SETUP.swap(true, core::sync::atomic::Ordering::SeqCst);

        // wrapping it at lest in a critical section.
        let is_configured = cortex_m::interrupt::free(|_| {
            let is_configured = EIC_SETUP.load(Ordering::SeqCst);
            EIC_SETUP.store(true, Ordering::SeqCst);
            is_configured
        });

        if !is_configured {
            // define clock generator 2 and connect it to the 8Mhz OSC
            let gclk2 = clocks
                .configure_gclk_divider_and_source(
                    ClockGenId::GCLK2,
                    1,
                    pac::gclk::genctrl::SRC_A::OSC8M,
                    false,
                )
                .unwrap();

            // "connect" eic clock to gclk2
            let eic_clock = clocks.eic(&gclk2).unwrap();

            // init External interrupt controller
            let mut eic = EIC::init(pm, eic_clock, eic);
            cortex_m::interrupt::free(|cs| {
                EIC.borrow(cs).replace(Some(eic));
            });

            unsafe {
                nvic.set_priority(interrupt::EIC, 2);
                NVIC::unmask(interrupt::EIC);
            }
        }
    }
}

fn get_pin_state(in0: bool, shift: u8) -> bool {
    let p = unsafe { &*pac::PORT::ptr() };
    if in0 {
        (p.in0.read().in_().bits() & (1 << shift)) != 0
    } else {
        (p.in1.read().in_().bits() & (1 << shift)) != 0
    }
}

impl ExtIntPin<pin_v2::PA05> {
    pub fn enable<M: PinMode>(pin: Pin<pin_v2::PA05, M>, cb: fn(bool) -> ()) -> Self {
        let mut extint = extInt::ExtInt5::new(pin.into_pull_down_interrupt());
        // configure ExtInt
        cortex_m::interrupt::free(|cs| {
            EIC.borrow(cs).borrow_mut().as_mut().map(|e| {
                extint.sense(e, Sense::BOTH);
                extint.filter(e, true);
                extint.enable_interrupt(e);
            });
        });

        let ext_id = 5;
        let state = get_pin_state(true, ext_id);
        Self {
            _p: PhantomData::default(),
            ext_id,
            state,
            cb,
        }
    }

    pub fn poll(&mut self) {
        cortex_m::interrupt::free(|cs| {
            self.state = get_pin_state(true, self.ext_id);
            let eic = unsafe { &*pac::EIC::ptr() };
            if self.is_triggered(eic) {
                (self.cb)(self.state);
                self.clear_flag(eic)
            }
        });
    }
    pub fn is_triggered(&self, eic: &RegisterBlock) -> bool {
        eic.intflag.read().extint5().bit_is_set()
    }
    pub fn clear_flag(&self, eic: &RegisterBlock) {
        eic.intflag.modify(|_, w| w.extint5().set_bit());
    }
}

impl ExtIntPin<pin_v2::PA06> {
    pub fn enable<M: PinMode>(pin: Pin<pin_v2::PA06, M>, cb: fn(bool) -> ()) -> Self {
        let mut extint = extInt::ExtInt6::new(pin.into_pull_down_interrupt());
        // configure ExtInt
        cortex_m::interrupt::free(|cs| {
            EIC.borrow(cs).borrow_mut().as_mut().map(|e| {
                extint.sense(e, Sense::BOTH);
                extint.filter(e, true);
                extint.enable_interrupt(e);
            });
        });

        let ext_id = 6;
        let state = get_pin_state(true, ext_id);

        Self {
            _p: PhantomData::default(),
            ext_id,
            state,
            cb,
        }
    }

    pub fn poll(&mut self) {
        cortex_m::interrupt::free(|cs| {
            self.state = get_pin_state(true, self.ext_id);
            let eic = unsafe { &*pac::EIC::ptr() };
            if self.is_triggered(eic) {
                (self.cb)(self.state);
                self.clear_flag(eic)
            }
        });
    }
    pub fn is_triggered(&self, eic: &RegisterBlock) -> bool {
        eic.intflag.read().extint6().bit_is_set()
    }
    pub fn clear_flag(&self, eic: &RegisterBlock) {
        eic.intflag.modify(|_, w| w.extint6().set_bit());
    }
}

impl ExtIntPin<pin_v2::PA07> {
    pub fn enable<M: PinMode>(pin: Pin<pin_v2::PA07, M>, cb: fn(bool) -> ()) -> Self {
        let mut extint = extInt::ExtInt7::new(pin.into_pull_down_interrupt());
        // configure ExtInt
        cortex_m::interrupt::free(|cs| {
            EIC.borrow(cs).borrow_mut().as_mut().map(|e| {
                extint.sense(e, Sense::BOTH);
                extint.filter(e, true);
                extint.enable_interrupt(e);
            });
        });

        let ext_id = 7;
        let state = get_pin_state(true, ext_id);

        Self {
            _p: PhantomData::default(),
            ext_id,
            state,
            cb,
        }
    }

    pub fn poll(&mut self) {
        cortex_m::interrupt::free(|cs| {
            self.state = get_pin_state(false, self.ext_id);
            let eic = unsafe { &*pac::EIC::ptr() };
            if self.is_triggered(eic) {
                (self.cb)(self.state);

                self.clear_flag(eic)
            }
        });
    }
    pub fn is_triggered(&self, eic: &RegisterBlock) -> bool {
        eic.intflag.read().extint7().bit_is_set()
    }
    pub fn clear_flag(&self, eic: &RegisterBlock) {
        eic.intflag.modify(|_, w| w.extint7().set_bit());
    }
}

impl ExtIntPin<pin_v2::PA09> {
    pub fn enable<M: PinMode>(pin: Pin<pin_v2::PA09, M>, cb: fn(bool) -> ()) -> Self {
        let mut extint = extInt::ExtInt9::new(pin.into_pull_down_interrupt());
        // configure ExtInt
        cortex_m::interrupt::free(|cs| {
            EIC.borrow(cs).borrow_mut().as_mut().map(|e| {
                extint.sense(e, Sense::BOTH);
                extint.filter(e, true);
                extint.enable_interrupt(e);
            });
        });

        let ext_id = 9;
        let state = get_pin_state(true, ext_id);

        Self {
            _p: PhantomData::default(),
            ext_id,
            state,
            cb,
        }
    }

    pub fn poll(&mut self) {
        cortex_m::interrupt::free(|cs| {
            self.state = get_pin_state(false, self.ext_id);
            let eic = unsafe { &*pac::EIC::ptr() };
            if self.is_triggered(eic) {
                (self.cb)(self.state);
                self.clear_flag(eic)
            }
        });
    }
    pub fn is_triggered(&self, eic: &RegisterBlock) -> bool {
        eic.intflag.read().extint9().bit_is_set()
    }
    pub fn clear_flag(&self, eic: &RegisterBlock) {
        eic.intflag.modify(|_, w| w.extint9().set_bit());
    }
}

impl ExtIntPin<pin_v2::PB09> {
    pub fn enable<M: PinMode>(pin: Pin<pin_v2::PB09, M>, cb: fn(bool) -> ()) -> Self {
        let mut extint = extInt::ExtInt9::new(pin.into_pull_down_interrupt());
        // configure ExtInt
        cortex_m::interrupt::free(|cs| {
            EIC.borrow(cs).borrow_mut().as_mut().map(|e| {
                extint.sense(e, Sense::BOTH);
                extint.filter(e, true);
                extint.enable_interrupt(e);
            });
        });

        let ext_id = 9;
        let state = get_pin_state(false, ext_id);
        Self {
            _p: PhantomData::default(),
            ext_id,
            state,
            cb,
        }
    }

    pub fn poll(&mut self) {
        cortex_m::interrupt::free(|cs| {
            self.state = get_pin_state(false, self.ext_id);
            let eic = unsafe { &*pac::EIC::ptr() };
            if self.is_triggered(eic) {
                self.state != self.state;
                (self.cb)(self.state);
                self.clear_flag(eic)
            }
        });
    }
    pub fn is_triggered(&self, eic: &RegisterBlock) -> bool {
        eic.intflag.read().extint9().bit_is_set()
    }
    pub fn clear_flag(&self, eic: &RegisterBlock) {
        eic.intflag.modify(|_, w| w.extint9().set_bit());
    }
}
