//! Working with timer counter hardware
use atsamd_hal::ehal::timer::CountDown;
use embedded_hal::timer::Periodic;
use hal::prelude::nb;

use crate::pac::tc3::COUNT32;
#[allow(unused)]
use crate::pac::{PM, TC3, TC4, TC5};
use hal::timer_params::TimerParams;

use hal::clock;
use hal::time::{Hertz, Nanoseconds};
use hal::timer_traits::InterruptDrivenTimer;

use xiao_m0::hal;

use cortex_m::asm::delay as cycle_delay;

// Note:
// TC3 + TC4 can be paired to make a 32-bit counter
// TC5 + TC6 can be paired to make a 32-bit counter

/// A generic hardware timer counter.
/// The counters are exposed in 16-bit mode only.
/// The hardware allows configuring the 8-bit mode
/// and pairing up some instances to run in 32-bit
/// mode, but that functionality is not currently
/// exposed by this hal implementation.
/// TimerCounter implements both the `Periodic` and
/// the `CountDown` embedded_hal timer traits.
/// Before a hardware timer can be used, it must first
/// have a clock configured.
pub struct TimerCounter32<TC: Count32> {
    freq: Hertz,
    tc: TC,
    periodic: bool,
}

/// This is a helper trait to make it easier to make most of the
/// TimerCounter impl generic.  It doesn't make too much sense to
/// to try to implement this trait outside of this module.
pub trait Count32 {
    fn count_32(&self) -> &COUNT32;
}

impl<TC> CountDown for TimerCounter32<TC>
where
    TC: Count32,
{
    type Time = Nanoseconds;

    fn start<T>(&mut self, timeout: T)
    where
        T: Into<Self::Time>,
    {
        let params = TimerParams::new_us(timeout, self.freq.0);
        let divider = params.divider;
        let cycles = params.cycles;

        let count = self.tc.count_32();

        // Disable the timer while we reconfigure it
        count.ctrla.modify(|_, w| w.enable().clear_bit());
        while count.status.read().syncbusy().bit_is_set() {}

        // Now that we have a clock routed to the peripheral, we
        // can ask it to perform a reset.
        count.ctrla.write(|w| w.swrst().set_bit());
        while count.status.read().syncbusy().bit_is_set() {}
        // the SVD erroneously marks swrst as write-only, so we
        // need to manually read the bit here
        while count.ctrla.read().bits() & 1 != 0 {}

        count.ctrlbset.write(|w| {
            // Count up when the direction bit is zero
            if self.periodic {
                // Periodic
                w.oneshot().clear_bit();
            }
            w.dir().clear_bit()
        });

        count.ctrla.modify(|_, w| {
            match divider {
                1 => w.prescaler().div1(),
                2 => w.prescaler().div2(),
                4 => w.prescaler().div4(),
                8 => w.prescaler().div8(),
                16 => w.prescaler().div16(),
                64 => w.prescaler().div64(),
                256 => w.prescaler().div256(),
                1024 => w.prescaler().div1024(),
                _ => unreachable!(),
            };
            // Enable Match Frequency Waveform generation
            w.enable().set_bit();
            w.runstdby().set_bit()
        });
    }

    fn wait(&mut self) -> nb::Result<(), void::Void> {
        let count = self.tc.count_32();
        if count.intflag.read().ovf().bit_is_set() {
            // Writing a 1 clears the flag
            count.intflag.modify(|_, w| w.ovf().set_bit());
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

macro_rules! tc {
    ($($TYPE:ident: ($TC:ident, $pm:ident, $clock:ident),)+) => {
        $(
pub type $TYPE = TimerCounter32<$TC>;

impl Count32 for $TC {
    fn count_32(&self) -> &COUNT32 {
        self.count32()
    }
}

impl TimerCounter32<$TC>
{
    pub fn get_count(&self) -> &COUNT32 {
        self.tc.count_32()
    }

    /// Configure this timer counter instance.
    /// The clock is obtained from the `GenericClockController` instance
    /// and its frequency impacts the resolution and maximum range of
    /// the timeout values that can be passed to the `start` method.
    /// Note that some hardware timer instances share the same clock
    /// generator instance and thus will be clocked at the same rate.
    pub fn $pm(clock: &clock::$clock, tc: $TC, pm: &mut PM, periodic: bool) -> Self {
        // this is safe because we're constrained to just the tc3 bit
        pm.apbcmask.modify(|_, w| w.$pm().set_bit());
        {
            let count = tc.count_32();

            // Disable the timer while we reconfigure it
            count.ctrla.modify(|_, w| w.enable().clear_bit());
            while count.status.read().syncbusy().bit_is_set() {}
        }
        Self {
            freq: clock.freq(),
            tc,
            periodic,
        }
    }
}
        )+
    }
}

tc! {
    TimerCounter32_3: (TC3, tc3_, Tcc2Tc3Clock),
    TimerCounter32_4: (TC4, tc4_, Tc4Tc5Clock),
    TimerCounter32_5: (TC5, tc5_, Tc4Tc5Clock),
}
