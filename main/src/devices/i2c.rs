use atsamd_hal::{
    clock::GenericClockController,
    gpio::{Floating, Input, Port},
    sercom::{I2CError, I2CMaster2, Sercom2Pad0, Sercom2Pad1},
};
use xiao_m0::{
    gpio::{self},
    i2c_master, pac,
    prelude::*,
};

pub struct I2c {
    main: I2CMaster2<Sercom2Pad0<gpio::Pa8<gpio::PfD>>, Sercom2Pad1<gpio::Pa9<gpio::PfD>>>,
}

impl I2c {
    pub fn init(
        clocks: &mut GenericClockController,
        sercom2: pac::SERCOM2,
        pm: &mut pac::PM,
        a4: gpio::Pa8<Input<Floating>>,
        a5: gpio::Pa9<Input<Floating>>,
        port: &mut Port,
    ) -> Self {
        let main = i2c_master(clocks, 1.mhz(), sercom2, pm, a4, a5, port);
        Self { main }
    }

    pub fn i2c_read_some(
        &mut self,
        address: u8,
        from: u8,
        count: usize,
        buffer: &mut [u8],
    ) -> Result<(), I2CError> {
        self.main.write(address, &[from])?;
        for i in 0..count {
            let mut res = [0u8];
            self.main.read(address, &mut res)?;
            buffer[i] = res[0];
        }
        Ok(())
    }
}
