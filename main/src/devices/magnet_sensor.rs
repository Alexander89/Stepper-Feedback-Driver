use atsamd_hal::sercom::I2CError;

use super::i2c::I2c;

enum ReadPhase {
    Query,
    Read1,
    Read2,
}

pub struct MagnetSensor {
    status: u8,
    pub detected: bool,
    low: bool,
    heigh: bool,
    pub raw_angle: u16,
    pub agc: u8,
    pub magnitude: u16,

    read_phase: ReadPhase,
    read_phase_buf: u16,
}

impl MagnetSensor {
    pub fn init() -> Self {
        Self {
            status: 0,
            detected: false,
            low: false,
            heigh: false,
            raw_angle: 0,
            agc: 0,
            magnitude: 0,
            read_phase: ReadPhase::Query,
            read_phase_buf: 0,
        }
    }

    pub fn poll(&mut self, i2c: &mut I2c) -> Result<(), I2CError> {
        let mut buf = [0u8; 5];

        self.detected = false;

        i2c.i2c_read_some(0x36, 0x0Bu8, 3, &mut buf)?;

        self.status = (buf[0] & 0b111000) >> 3;
        self.detected = (self.status & 0b100) != 0;
        self.low = (self.status & 0b10) != 0;
        self.heigh = (self.status & 0b1) != 0;
        self.raw_angle = (((buf[1] as u16) << 8) + (buf[2] as u16)) & 0x0FFF;

        Ok(())
    }

    pub fn stepwise_read(&mut self, i2c: &mut I2c) -> Result<bool, I2CError> {
        match self.read_phase {
            ReadPhase::Query => {
                i2c.i2c_query(0x36, 0x0Cu8)?;

                self.read_phase = ReadPhase::Read1;
                Ok(false)
            }
            ReadPhase::Read1 => {
                self.read_phase_buf = i2c.i2c_read_one(0x36)? as u16;

                self.read_phase = ReadPhase::Read2;
                Ok(false)
            }
            ReadPhase::Read2 => {
                let v = i2c.i2c_read_one(0x36)? as u16;

                self.raw_angle = ((self.read_phase_buf << 8) + v) & 0x0FFF;
                self.read_phase = ReadPhase::Query;
                Ok(true)
            }
        }
    }

    pub fn query(&mut self, i2c: &mut I2c) -> Result<(), I2CError> {
        i2c.i2c_query(0x36, 0x0Cu8)
    }

    pub fn read(&mut self, i2c: &mut I2c) -> Result<(), I2CError> {
        let mut buf = [0u8; 4];

        i2c.i2c_read(0x36, 2, &mut buf)?;

        self.raw_angle = (((buf[0] as u16) << 8) + (buf[1] as u16)) & 0x0FFF;
        Ok(())
    }

    pub fn poll_setup(&mut self, i2c: &mut I2c) -> Result<(), I2CError> {
        let mut buf = [0u8; 5];
        self.agc = 0;
        self.magnitude = 0;

        i2c.i2c_read_some(0x36, 0x1Au8, 3, &mut buf)?;

        self.agc = buf[0] as u8;
        self.magnitude = (((buf[1] as u16) << 8) + (buf[2] as u16)) & 0x0FFF;

        i2c.i2c_read_some(0x36, 0x0Bu8, 1, &mut buf)?;
        self.detected = false;
        self.status = (buf[0] & 0b111000) >> 3;
        self.detected = (self.status & 0b100) != 0;
        self.low = (self.status & 0b10) != 0;
        self.heigh = (self.status & 0b1) != 0;
        Ok(())
    }
}
