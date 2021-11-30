use core::marker::PhantomData;
use core::sync::atomic::AtomicBool;
use core::{cell::RefCell, sync::atomic::Ordering};
use core::borrow::BorrowMut;

use cortex_m::interrupt::{CriticalSection, Mutex};
use embedded_hal::digital::v2::PinState;
use hal::clock::GClock;
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
pub fn init(
    cs: &CriticalSection,
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
                pac::gclk::genctrl::SRC_A::OSC32K,
                false,
            )
            .unwrap();

        // "connect" eic clock to gclk2
        let eic_clock = clocks.eic(&gclk2).unwrap();

        // init External interrupt controller
        EIC.borrow(cs).replace(Some(EIC::init(pm, eic_clock, eic)));

        unsafe {
            nvic.set_priority(interrupt::EIC, 1);
            NVIC::unmask(interrupt::EIC);
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


macro_rules! eip {
    (
        $Pad:ident, 
        $num:expr,
        $is_in0:expr
     ) => {
paste::item! {
    impl ExtIntPin<pin_v2::$Pad> {
        pub fn enable<M: PinMode>(pin: Pin<pin_v2::$Pad, M>, cb: fn(bool) -> ()) -> Self {
            let mut extint = extInt::[<ExtInt $num>]::new(pin.into_pull_down_interrupt());
            // configure ExtInt
            cortex_m::interrupt::free(|cs| {
                EIC.borrow(cs).borrow_mut().as_mut().map(|e| {
                    extint.sense(e, Sense::BOTH);
                    extint.filter(e, true);
                    extint.enable_interrupt(e);
                });
            });

            let ext_id = $num;
            let state = get_pin_state($is_in0, ext_id);
            Self {
                _p: PhantomData::default(),
                ext_id,
                state,
                cb,
            }
        }

        pub fn poll(&mut self) {
            let eic = unsafe { &*pac::EIC::ptr() };
            if self.is_triggered(eic) {
                self.execute();
                cortex_m::interrupt::free(|_| {
                    self.clear_flag(eic)
                });
            }
        }

        pub fn execute(&mut self) {
            self.state = get_pin_state($is_in0, self.ext_id);
            (self.cb)(self.state);
        }

        fn is_triggered(&self, eic: &RegisterBlock) -> bool {
            eic.intflag.read().[<extint $num>]().bit_is_set()
        }
        fn clear_flag(&self, eic: &RegisterBlock) {
            eic.intflag.modify(|_, w| w.[<extint $num>]().set_bit());
        }
    }
}
}
}

eip!(PA05, 5, true);
eip!(PA06, 6, true);
eip!(PA07, 7, true);
eip!(PA09, 9, true);
eip!(PB09, 9, false);
