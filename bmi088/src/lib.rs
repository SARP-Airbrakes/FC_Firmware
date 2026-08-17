#![no_std]

use embedded_hal::i2c::{I2c, SevenBitAddress};


pub mod regs;
mod measurements;

pub use measurements::*;

pub struct Bmi088<I> {
    i2c: I,
    sdo1_high: bool, /* for ACC address */
    sdo2_high: bool, /* for GYRO address */
}

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    Unidentified
}

enum Bmi088Device {
    Acc,
    Gyro
}

impl<I, E> Bmi088<I>
where
    I: I2c<SevenBitAddress, Error = E>
{
    pub fn new(i2c: I) -> Self {
        Bmi088 {
            i2c,
            sdo1_high: false,
            sdo2_high: false,
        }
    }

    pub fn with_sdo1_high(mut self) -> Self {
        self.sdo1_high = true;
        self
    }

    pub fn with_sdo2_high(mut self) -> Self {
        self.sdo2_high = true;
        self
    }

    pub fn destroy(self) -> I {
        self.i2c
    }

    pub fn read_acc(&mut self) -> Result<Bmi088Acceleration, Error<E>> {
        let mut data = [0u8; 6];
        self.read_bytes(Bmi088Device::Acc, regs::BMI088_ACC_X_LSB, &mut data).map_err(Error::I2c)?;
        let x = i16::from_le_bytes([data[0], data[1]]);
        let y = i16::from_le_bytes([data[2], data[3]]);
        let z = i16::from_le_bytes([data[4], data[5]]);
        Ok(Bmi088Acceleration::new(x, y, z))
    }

    pub fn init(&mut self) -> Result<(), Error<E>> {
        let byte = self.read_u8(Bmi088Device::Acc, regs::BMI088_ACC_CHIP_ID).map_err(Error::I2c)?;
        #[cfg(feature = "defmt")]
        {
            defmt::trace!("Received {} for ACC_CHIP_ID, expecting 0x1e", byte);
        }
        if byte != 0x1e {
            return Err(Error::Unidentified);
        }

        // See section 3
        // delay.delay_ms(1);
        self.write_u8(Bmi088Device::Acc, regs::BMI088_ACC_PWR_CTRL, 0x04)
            .map_err(Error::I2c)?; 
        // delay.delay_ms(50);
        
        Ok(())
    }

    fn i2c_addr(&self, device: Bmi088Device) -> u8 {
        match device {
            Bmi088Device::Acc => if self.sdo1_high {
                regs::BMI088_ACC_ADDRESS_HIGH 
            } else {
                regs::BMI088_ACC_ADDRESS_LOW 
            },
            Bmi088Device::Gyro => if self.sdo2_high {
                regs::BMI088_GYRO_ADDRESS_HIGH
            } else {
                regs::BMI088_GYRO_ADDRESS_LOW
            },
        }
    }

    fn read_u8(&mut self, device: Bmi088Device, reg: u8) -> Result<u8, E> {
        let mut out: [u8; 1] = [0; 1];
        self.i2c.write_read(self.i2c_addr(device), &[reg], &mut out)?;
        Ok(out[0])
    }

    fn read_bytes(&mut self, device: Bmi088Device, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(self.i2c_addr(device), &[reg], buf)
    }

    fn write_u8(&mut self, device: Bmi088Device, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(self.i2c_addr(device), &[reg, value])?;
        Ok(())
    }
}

