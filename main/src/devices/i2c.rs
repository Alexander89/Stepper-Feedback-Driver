use atsamd_hal::{
    clock::GenericClockController,
    gpio::{
        self,
        v2::{AnyPin, Pin},
        Floating, Input,
    },
    pac,
    prelude::*,
    sercom::{
        v2::{Pad0, Pad1},
        CompatiblePad, I2CError, I2CMaster0, I2CMaster2, Sercom2Pad0, Sercom2Pad1,
    },
    target_device::SERCOM2,
    time::Hertz,
};
use cortex_m::interrupt;
use xiao_m0::{i2c_master, Scl, Sda, I2C};

pub struct I2c {
    main: I2C,
}

impl I2c {
    pub fn init(
        clocks: &mut GenericClockController,
        sercom: pac::SERCOM0,
        pm: &mut pac::PM,
        sda: impl Into<Sda>,
        scl: impl Into<Scl>,
    ) -> Self {
        let gclk0 = clocks.gclk0();
        let clock = &clocks.sercom0_core(&gclk0).unwrap();
        let freq: Hertz = 1.mhz().into();
        let main = I2CMaster0::new(clock, freq, sercom, pm, sda.into(), scl.into());
        Self { main }
    }

    pub fn i2c_read_some(
        &mut self,
        address: u8,
        from: u8,
        count: usize,
        buffer: &mut [u8],
    ) -> Result<(), I2CError> {
        interrupt::free(|_| self.main.write(address, &[from]))?;
        for i in 0..count {
            let mut res = [0u8];
            interrupt::free(|_| self.main.read(address, &mut res))?;
            buffer[i] = res[0];
        }
        Ok(())
    }

    pub fn i2c_query(&mut self, address: u8, from: u8) -> Result<(), I2CError> {
        interrupt::free(|_| self.main.write(address, &[from]))?;
        Ok(())
    }

    pub fn i2c_read(
        &mut self,
        address: u8,
        count: usize,
        buffer: &mut [u8],
    ) -> Result<(), I2CError> {
        for i in 0..count {
            let mut res = [0u8];
            interrupt::free(|_| self.main.read(address, &mut res))?;
            buffer[i] = res[0];
        }
        Ok(())
    }
}
