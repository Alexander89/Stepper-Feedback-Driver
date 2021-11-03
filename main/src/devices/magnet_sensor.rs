use atsamd_hal::sercom::I2CError;

use super::i2c::I2c;

pub struct MagnetSensor {
    status: u8,
    detected: bool,
    low: bool,
    heigh: bool,
    raw_angle: u16,
    angle: u16,
    agc: u8,
    magnitude: u16,
}

impl MagnetSensor {
    pub fn init() -> Self {
        Self {
            status: 0,
            detected: false,
            low: false,
            heigh: false,
            raw_angle: 0,
            angle: 0,
            agc: 0,
            magnitude: 0,
        }
    }

    pub fn poll(&mut self, i2c: &mut I2c) -> Result<(), I2CError> {
        let mut buf = [0u8; 5];

        self.detected = false;

        i2c.i2c_read_some(0x36, 0x0Bu8, 5, &mut buf)?;

        self.status = (buf[0] & 0b111000) >> 3;
        self.detected = (self.status & 0b100) != 0;
        self.low = (self.status & 0b10) != 0;
        self.heigh = (self.status & 0b1) != 0;
        self.raw_angle = (((buf[1] as u16) << 8) + (buf[2] as u16)) & 0x0FFF;
        self.angle = (((buf[3] as u16) << 8) + (buf[4] as u16)) & 0x0FFF;

        Ok(())
    }

    pub fn poll_setup(&mut self, i2c: &mut I2c) -> Result<(), I2CError> {
        let mut buf = [0u8; 5];

        self.agc = 0;
        self.magnitude = 0;

        i2c.i2c_read_some(0x36, 0x1Au8, 3, &mut buf)?;

        self.agc = buf[0] as u8;
        self.magnitude = (((buf[1] as u16) << 8) + (buf[2] as u16)) & 0x0FFF;
        Ok(())
    }
}

// if detected {

//     serial_write(b"a: ");
//     delay.delay_ms(2u16);
//     serial_write_num(angle);
//     delay.delay_ms(2u16);

//     serial_write(b" agc: ");
//     delay.delay_ms(2u16);
//     serial_write_num(agc);
//     delay.delay_ms(2u16);

//     serial_write(b" mag: ");
//     delay.delay_ms(2u16);
//     serial_write_num(magnitude);
//     delay.delay_ms(2u16);

//     // serial_write(b" a: ");
//     // delay.delay_ms(2u16);
//     // let (len, bytes) = num_to_string(angle);
//     // serial_write_len(&bytes, len);
//     // delay.delay_ms(2u16);

//     serial_write(b"\r\n");
//     delay.delay_ms(2u16);
// }
