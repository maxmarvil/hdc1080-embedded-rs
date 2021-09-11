#![deny(unsafe_code)]
#![no_std]
use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Write, WriteRead, Read},
};

const I2C_ADDRESS: u8 = 0x40;


/// Error.
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// Device is not calibrated.
    Uncalibrated,
    /// Underlying bus error.
    Bus(E),
    /// Checksum mismatch.
    Checksum,
}

/// Resolution of temperature
#[derive(Debug, Clone, Copy)]
pub enum TResolution {
    /// 11 bits
    _11,
    /// 14 bits
    _14,
}

/// Resolution of humidity
#[derive(Debug, Clone, Copy)]
pub enum HResolution {
    /// 8 bits
    _8,
    /// 11 bits
    _11,
    /// 14 bits
    _14,
}

struct Register;

impl Register {
    const TEMPERATURE: u8 = 0x00;
    const HUMIDITY: u8 = 0x01;
    const CONFIGURATION: u8 = 0x02;
    const SERIAL_ID1: u8 = 0xFB;
    const SERIAL_ID2: u8 = 0xFC;
    const SERIAL_ID3: u8 = 0xFD;
    const MANUFACTURER: u8 = 0xFE;
    const DEVICE_ID: u8 = 0xFF;
}

struct ConfigBitFlags;

impl ConfigBitFlags {
    const RST: u16 = 0b1000_0000_0000_0000;
    const HEAT: u16 = 0b0010_0000_0000_0000;
    const MODE: u16 = 0b0001_0000_0000_0000;
    const BTST: u16 = 0b0000_1000_0000_0000;
    const T_MODE: u16 = 0b0000_0100_0000_0000;
    const H_MODE9: u16 = 0b000_0010_0000_0000;
    const H_MODE8: u16 = 0b000_0001_0000_0000;
}

pub struct Hdc1080<I2C, D> {
    i2c: I2C,
    config: u16,
    delay: D,
}

impl<I2C, D, E> Hdc1080<I2C, D> where
    I2C: WriteRead<Error = E> + Write<Error = E>+ Read<Error = E>,
    D: DelayMs<u16>,
{
    /// New HDC1080 device from an I2C peripheral.
    /// temperature 11bit humidity 11b
    pub fn new(i2c: I2C, delay: D ) -> Result<Self, Error<E>> {
        let dev = Self {
            i2c: i2c,
            delay: delay,
            config:0x00|ConfigBitFlags::T_MODE|ConfigBitFlags::MODE| ConfigBitFlags::H_MODE8 & !ConfigBitFlags::H_MODE9
        };

        Ok(dev)
    }

    /// Init with default config
    pub fn init(& mut self) -> Result<(), E>{
        self.set_config(self.config).unwrap_or_default();
        Ok(())
    }

    /// Soft resets the sensor.
    pub fn reset(&mut self) -> Result<(), E> {
        // Send soft reset command
        let bytes:[u8;2] = ConfigBitFlags::RST.to_be_bytes();
        self.i2c.write(I2C_ADDRESS, &[Register::CONFIGURATION, bytes[0]]).unwrap_or_default();

        self.delay.delay_ms(10);

        Ok(())
    }

    /// Set temperature resolution
    pub fn set_t_resolution(& mut self, resolution: TResolution) -> Result<(), E> {
        match resolution {
            TResolution::_11 => self.set_config(self.config | ConfigBitFlags::T_MODE).unwrap_or_default(),
            TResolution::_14 => self.set_config(self.config & !ConfigBitFlags::T_MODE).unwrap_or_default(),
        }
        Ok(())
    }

    /// Set humidity resolution
    pub fn set_h_resolution(& mut self, resolution: HResolution) -> Result<(), E> {
        match resolution {
            HResolution::_8 => self.set_config(self.config & !ConfigBitFlags::H_MODE8 | ConfigBitFlags::H_MODE9).unwrap_or_default(),
            HResolution::_11 => self.set_config(self.config | ConfigBitFlags::H_MODE8 & !ConfigBitFlags::H_MODE9).unwrap_or_default(),
            HResolution::_14 => self.set_config(self.config & !ConfigBitFlags::H_MODE8 & !ConfigBitFlags::H_MODE9).unwrap_or_default(),
        }
        Ok(())
    }

    /// Read config from register
    pub fn read_config(& mut self) -> Result<u16, E>{
        let mut current_config:[u8;2] = [0,0];
        let result = self.i2c.write_read(I2C_ADDRESS, &[Register::CONFIGURATION], &mut current_config).unwrap_or_default();
        
        Ok(u16::from_be_bytes(current_config))
    }

    /// Set config from u16
    pub fn set_config(& mut self, bits:u16) -> Result<(), E> {
        self.config = bits;
        let conf_u8: [u8;2] = self.config.to_be_bytes();
        self.i2c.write(I2C_ADDRESS, &[Register::CONFIGURATION, conf_u8[0]]).unwrap_or_default();
        self.delay.delay_ms(10);
        Ok(())
    }

    /// Read temperature and humidity
    pub fn read(& mut self) -> Result<(f32, f32),Error<E>> {
        let mut buf:[u8;4] = [0,0,0,0];
        let t_raw_u16:u16 ;
        let h_raw_u16:u16 ;
        let temper:f32;
        let humm:f32;

        self.i2c.write(I2C_ADDRESS, &[Register::TEMPERATURE]).unwrap_or_default();
        self.delay.delay_ms(20);
        self.i2c.read(I2C_ADDRESS,&mut buf).unwrap_or_default();
        t_raw_u16 = u16::from_be_bytes([buf[0], buf[1]]);
        temper = f32::from(t_raw_u16) / 65536.0 * 165.0 - 40.0;

        if self.config & ConfigBitFlags::MODE != 0 {
            h_raw_u16 = u16::from_be_bytes([buf[2], buf[3]]);
            humm = f32::from(h_raw_u16) / 65536.0 * 100.0;
        } else {
            humm = self.humidity().unwrap_or_default();
        }

        Ok((temper, humm))
    }

    /// Temperature only
    pub fn temperature(& mut self) -> Result<f32,Error<E>> {
        let mut buf:[u8;2] = [0,0];
        let result:u16 ;
        let temper:f32;

        self.i2c.write(I2C_ADDRESS, &[Register::TEMPERATURE]).unwrap_or_default();
        self.delay.delay_ms(20);
        self.i2c.read(I2C_ADDRESS,&mut buf).unwrap_or_default();

        result = u16::from_be_bytes(buf);
        temper = f32::from(result) / 65536.0 * 165.0 - 40.0;
        Ok(temper)
    }

    /// Humidity only
    pub fn humidity(& mut self) -> Result<f32,Error<E>> {
        let mut buf:[u8;2] = [0,0];
        let result:u16 ;
        let humid:f32;

        self.i2c.write(I2C_ADDRESS, &[Register::HUMIDITY]).unwrap_or_default();
        self.delay.delay_ms(20);
        self.i2c.read(I2C_ADDRESS,&mut buf).unwrap_or_default();

        result = u16::from_be_bytes(buf);
        humid = f32::from(result) / 65536.0 * 100.0;
        Ok(humid)
    }

    /// Device ID. Expect u16 0x1050
    pub fn get_device_id(&mut self) -> Result<u16, Error<E>> {
        let mut buf:[u8;2] = [0,0];
        let result:u16 ;
        self.i2c.write_read(I2C_ADDRESS, &[Register::DEVICE_ID],&mut buf).unwrap_or_default();
        result = ((buf[0] as u16)<<8)|(buf[1] as u16);
        Ok(result)
    }

    /// Manufacturer ID. Expect u16 0x5449
    pub fn get_man_id(&mut self) -> Result<u16, Error<E>> {
        let mut buf = [0u8; 2];
        let result:u16 ;
        self.i2c.write_read(I2C_ADDRESS, &[Register::MANUFACTURER],&mut buf).unwrap_or_default();
        result = ((buf[0] as u16)<<8)|(buf[1] as u16);
        Ok(result)
    }

    /// Serial ID. Expect [u16;3]
    pub fn get_serial_id(&mut self) -> Result<[u16;3], Error<E>> {
        let mut buf1 = [0u8; 2];
        let mut buf2 = [0u8; 2];
        let mut buf3 = [0u8; 2];
        let mut result: [u16;3] = [0,0,0] ;
        self.i2c.write_read(I2C_ADDRESS,&[Register::SERIAL_ID1], &mut buf1 ).unwrap_or_default();
        self.i2c.write_read(I2C_ADDRESS,&[Register::SERIAL_ID2], &mut buf2 ).unwrap_or_default();
        self.i2c.write_read(I2C_ADDRESS,&[Register::SERIAL_ID3], &mut buf3 ).unwrap_or_default();
        result[0] = u16::from_be_bytes(buf1);
        result[1] = u16::from_be_bytes(buf2);
        result[2] = u16::from_be_bytes(buf3);
        Ok(result)
    }

    /// Returns true if battery voltage is under 2.8v
    pub fn battery_low(&mut self) -> Result<bool, E> {
        let config = self.read_config().unwrap_or_default();
        Ok((config & ConfigBitFlags::BTST) == ConfigBitFlags::BTST)
    }

}
