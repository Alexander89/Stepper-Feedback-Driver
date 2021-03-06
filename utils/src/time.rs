//! Time units

use atsamd_hal::time as ats_time;
use core::ops::{Add, AddAssign, Sub};

// Frequency based

/// Bits per second
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Bps(pub u32);

/// Hertz
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Hertz(pub u32);

/// KiloHertz
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct KiloHertz(pub u32);

/// MegaHertz
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct MegaHertz(pub u32);

// Period based

/// Seconds
#[derive(Clone, Copy, Eq, Debug)]
pub struct Seconds(pub u32);

/// Milliseconds
#[derive(Clone, Copy, Eq, Debug)]
pub struct Milliseconds(pub u32);

/// Microseconds
#[derive(Clone, Copy, Eq, Debug)]
pub struct Microseconds(pub u32);

/// Nanoseconds
#[derive(Clone, Copy, Eq, Debug)]
pub struct Nanoseconds(pub u32);

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> Bps;

    /// Wrap in `Hertz`
    fn hz(self) -> Hertz;

    /// Wrap in `KiloHertz`
    fn khz(self) -> KiloHertz;

    /// Wrap in `MegaHertz`
    fn mhz(self) -> MegaHertz;

    /// Wrap in `Seconds`
    fn s(self) -> Seconds;

    /// Wrap in `Milliseconds`
    fn ms(self) -> Milliseconds;

    /// Wrap in `Microseconds`
    fn us(self) -> Microseconds;

    /// Wrap in `NanoSeconds`
    fn ns(self) -> Nanoseconds;
}

impl U32Ext for u32 {
    // Frequency based

    fn bps(self) -> Bps {
        Bps(self)
    }

    fn hz(self) -> Hertz {
        Hertz(self)
    }

    fn khz(self) -> KiloHertz {
        KiloHertz(self)
    }

    fn mhz(self) -> MegaHertz {
        MegaHertz(self)
    }

    // Period based

    fn s(self) -> Seconds {
        Seconds(self)
    }

    fn ms(self) -> Milliseconds {
        Milliseconds(self)
    }

    fn us(self) -> Microseconds {
        Microseconds(self)
    }

    fn ns(self) -> Nanoseconds {
        Nanoseconds(self)
    }
}

// Frequency based

impl From<KiloHertz> for Hertz {
    fn from(item: KiloHertz) -> Self {
        Hertz(item.0 * 1_000)
    }
}

impl From<MegaHertz> for Hertz {
    fn from(item: MegaHertz) -> Self {
        Hertz(item.0 * 1_000_000)
    }
}

impl From<MegaHertz> for KiloHertz {
    fn from(item: MegaHertz) -> Self {
        KiloHertz(item.0 * 1_000)
    }
}

impl From<Hertz> for KiloHertz {
    fn from(item: Hertz) -> Self {
        KiloHertz(item.0 / 1_000)
    }
}

impl From<Hertz> for MegaHertz {
    fn from(item: Hertz) -> Self {
        MegaHertz(item.0 / 1_000_000)
    }
}

impl From<KiloHertz> for MegaHertz {
    fn from(item: KiloHertz) -> Self {
        MegaHertz(item.0 / 1_000)
    }
}

// Period based

impl From<Seconds> for Milliseconds {
    fn from(item: Seconds) -> Self {
        Milliseconds(item.0 * 1_000)
    }
}
impl From<Seconds> for Microseconds {
    fn from(item: Seconds) -> Self {
        Microseconds(item.0 * 1_000_000)
    }
}

impl From<Seconds> for Nanoseconds {
    fn from(item: Seconds) -> Self {
        Nanoseconds(item.0 * 1_000_000_000)
    }
}

impl From<Milliseconds> for Microseconds {
    fn from(item: Milliseconds) -> Self {
        Microseconds(item.0 * 1_000)
    }
}

impl From<Microseconds> for Nanoseconds {
    fn from(item: Microseconds) -> Self {
        Nanoseconds(item.0 * 1_000)
    }
}

impl From<Milliseconds> for Seconds {
    fn from(item: Milliseconds) -> Self {
        Seconds(item.0 / 1_000)
    }
}

impl From<Microseconds> for Seconds {
    fn from(item: Microseconds) -> Self {
        Seconds(item.0 / 1_000_000)
    }
}

impl From<Microseconds> for Milliseconds {
    fn from(item: Microseconds) -> Self {
        Milliseconds(item.0 / 1_000)
    }
}

impl From<Milliseconds> for Nanoseconds {
    fn from(item: Milliseconds) -> Self {
        Nanoseconds(item.0 * 1_000_000)
    }
}

// Frequency <-> Period

impl From<Nanoseconds> for Hertz {
    fn from(item: Nanoseconds) -> Self {
        Hertz(1_000_000_000_u32 / item.0)
    }
}

impl From<Microseconds> for Hertz {
    fn from(item: Microseconds) -> Self {
        Hertz(1_000_000_u32 / item.0)
    }
}

impl From<Nanoseconds> for KiloHertz {
    fn from(item: Nanoseconds) -> Self {
        KiloHertz(1_000_000_u32 / item.0)
    }
}

impl From<Nanoseconds> for MegaHertz {
    fn from(item: Nanoseconds) -> Self {
        MegaHertz(1_000_u32 / item.0)
    }
}

impl From<Hertz> for Microseconds {
    fn from(item: Hertz) -> Self {
        Microseconds(1_000_000_u32 / item.0)
    }
}

impl From<Hertz> for Nanoseconds {
    fn from(item: Hertz) -> Self {
        Nanoseconds(1_000_000_000u32 / item.0)
    }
}

impl From<KiloHertz> for Nanoseconds {
    fn from(item: KiloHertz) -> Self {
        Nanoseconds(1_000_000u32 / item.0)
    }
}

impl From<MegaHertz> for Nanoseconds {
    fn from(item: MegaHertz) -> Self {
        Nanoseconds(1_000u32 / item.0)
    }
}

impl<T> Add<T> for Seconds
where
    T: Into<Seconds>,
{
    type Output = Seconds;
    fn add(self, item: T) -> Self::Output {
        Seconds(self.0 + item.into().0)
    }
}
impl<T> Add<T> for Milliseconds
where
    T: Into<Milliseconds>,
{
    type Output = Milliseconds;
    fn add(self, item: T) -> Self::Output {
        Milliseconds(self.0 + item.into().0)
    }
}
impl<T> Add<T> for Microseconds
where
    T: Into<Microseconds>,
{
    type Output = Microseconds;
    fn add(self, item: T) -> Self::Output {
        Microseconds(self.0 + item.into().0)
    }
}
impl<T> Add<T> for Nanoseconds
where
    T: Into<Nanoseconds>,
{
    type Output = Nanoseconds;
    fn add(self, item: T) -> Self::Output {
        Nanoseconds(self.0 + item.into().0)
    }
}

impl<T> AddAssign<T> for Microseconds
where
    T: Into<Microseconds>,
{
    fn add_assign(&mut self, rhs: T) {
        self.0 += rhs.into().0;
    }
}

impl<T> Sub<T> for Seconds
where
    T: Into<Seconds>,
{
    type Output = Seconds;
    fn sub(self, item: T) -> Self::Output {
        Seconds(self.0 - item.into().0)
    }
}
impl<T> Sub<T> for Milliseconds
where
    T: Into<Milliseconds>,
{
    type Output = Milliseconds;
    fn sub(self, item: T) -> Self::Output {
        Milliseconds(self.0 - item.into().0)
    }
}
impl<T> Sub<T> for Microseconds
where
    T: Into<Microseconds>,
{
    type Output = Microseconds;
    fn sub(self, item: T) -> Self::Output {
        let o: Microseconds = item.into();
        if o.0 > self.0 {
            0.us()
        } else {
            Microseconds(self.0 - o.0)
        }
    }
}
impl<T> Sub<T> for Nanoseconds
where
    T: Into<Nanoseconds>,
{
    type Output = Nanoseconds;
    fn sub(self, item: T) -> Self::Output {
        Nanoseconds(self.0 - item.into().0)
    }
}

impl<T> PartialEq<T> for Seconds
where
    T: Into<Seconds> + Copy,
{
    fn eq(&self, other: &T) -> bool {
        let o: Seconds = (*other).into();
        self.0 == o.0
    }
}

impl<T> PartialOrd<T> for Seconds
where
    T: Into<Seconds> + Copy,
{
    fn partial_cmp(&self, other: &T) -> Option<core::cmp::Ordering> {
        let o: Seconds = (*other).into();
        self.0.partial_cmp(&o.0)
    }
}

impl<T> PartialEq<T> for Milliseconds
where
    T: Into<Milliseconds> + Copy,
{
    fn eq(&self, other: &T) -> bool {
        let o: Milliseconds = (*other).into();
        self.0 == o.0
    }
}

impl<T> PartialOrd<T> for Milliseconds
where
    T: Into<Milliseconds> + Copy,
{
    fn partial_cmp(&self, other: &T) -> Option<core::cmp::Ordering> {
        let o: Milliseconds = (*other).into();
        self.0.partial_cmp(&o.0)
    }
}

impl<T> PartialEq<T> for Microseconds
where
    T: Into<Microseconds> + Copy,
{
    fn eq(&self, other: &T) -> bool {
        let o: Microseconds = (*other).into();
        self.0 == o.0
    }
}

impl<T> PartialOrd<T> for Microseconds
where
    T: Into<Microseconds> + Copy,
{
    fn partial_cmp(&self, other: &T) -> Option<core::cmp::Ordering> {
        let o: Microseconds = (*other).into();
        self.0.partial_cmp(&o.0)
    }
}

impl<T> PartialEq<T> for Nanoseconds
where
    T: Into<Nanoseconds> + Copy,
{
    fn eq(&self, other: &T) -> bool {
        let o: Nanoseconds = (*other).into();
        self.0 == o.0
    }
}

impl<T> PartialOrd<T> for Nanoseconds
where
    T: Into<Nanoseconds> + Copy,
{
    fn partial_cmp(&self, other: &T) -> Option<core::cmp::Ordering> {
        let o: Nanoseconds = (*other).into();
        self.0.partial_cmp(&o.0)
    }
}

// to ats_time

impl From<Seconds> for ats_time::Seconds {
    fn from(item: Seconds) -> Self {
        ats_time::Seconds(item.0)
    }
}
impl From<Milliseconds> for ats_time::Milliseconds {
    fn from(item: Milliseconds) -> Self {
        ats_time::Milliseconds(item.0)
    }
}

impl From<Microseconds> for ats_time::Microseconds {
    fn from(item: Microseconds) -> Self {
        ats_time::Microseconds(item.0)
    }
}

impl From<Nanoseconds> for ats_time::Nanoseconds {
    fn from(item: Nanoseconds) -> Self {
        ats_time::Nanoseconds(item.0)
    }
}

impl From<Bps> for ats_time::Bps {
    fn from(item: Bps) -> Self {
        ats_time::Bps(item.0)
    }
}
impl From<Hertz> for ats_time::Hertz {
    fn from(item: Hertz) -> Self {
        ats_time::Hertz(item.0)
    }
}

impl From<KiloHertz> for ats_time::KiloHertz {
    fn from(item: KiloHertz) -> Self {
        ats_time::KiloHertz(item.0)
    }
}

impl From<MegaHertz> for ats_time::MegaHertz {
    fn from(item: MegaHertz) -> Self {
        ats_time::MegaHertz(item.0)
    }
}

#[cfg(test)]
mod tests {
    use crate::time::*;

    #[test]
    fn convert_us_to_hz() {
        let as_us: Microseconds = 3.hz().into();
        assert_eq!(as_us.0, 333_333_u32);
    }

    #[test]
    fn convert_ms_to_us() {
        let as_us: Microseconds = 3.ms().into();
        assert_eq!(as_us.0, 3_000_u32);
    }

    #[test]
    fn convert_mhz_to_hz() {
        let as_hz: Hertz = 48.mhz().into();
        assert_eq!(as_hz.0, 48_000_000_u32);
    }

    #[test]
    fn convert_hz_to_ns() {
        let as_ns: Nanoseconds = 3.mhz().into();
        assert_eq!(as_ns.0, 333_u32);
    }

    #[test]
    fn convert_hz_to_ns_even() {
        let as_ns: Nanoseconds = 2.mhz().into();
        assert_eq!(as_ns.0, 500_u32);
    }

    // Test Add
    #[test]
    fn add_sec_to_nano() {
        let ns: Nanoseconds = 123.ns() + 1.s();
        assert_eq!(ns.0, 1_000_000_123_u32);
    }

    #[test]
    fn add_sec_to_micro() {
        let us: Microseconds = 123.us() + 1.s();
        assert_eq!(us.0, 1_000_123_u32);
    }

    #[test]
    fn add_sec_to_milli() {
        let ms: Milliseconds = 123.ms() + 1.s();
        assert_eq!(ms.0, 1_123_u32);
    }

    #[test]
    fn add_milli_to_sec() {
        let ms: Seconds = 1.s() + 123.ms();
        assert_eq!(ms.0, 1u32);
    }

    // Test Sub
    #[test]
    fn sub_sec_to_nano() {
        let ns: Nanoseconds = 1_000_123.ns() - 1.ms();
        assert_eq!(ns.0, 123_u32);
    }

    #[test]
    fn sub_sec_to_micro() {
        let us: Microseconds = 1_000_123.us() - 1.s();
        assert_eq!(us.0, 123_u32);
    }

    #[test]
    fn sub_sec_to_milli() {
        let ms: Milliseconds = 1_000.ms() - 2_345.us();
        assert_eq!(ms.0, 998_u32);
    }

    #[test]
    fn sub_milli_to_sec() {
        let s: Seconds = 10.s() - 1_230.ms();
        assert_eq!(s.0, 9u32);
    }
}
