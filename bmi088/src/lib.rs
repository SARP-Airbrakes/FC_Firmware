#![no_std]

use embedded_hal_async::{delay::DelayNs, i2c::{I2c, SevenBitAddress}};

mod regs;
mod config;
mod measurements;

pub use measurements::*;
pub use config::*;

pub struct Bmi088<I> {
    i2c: I,
    sdo1_high: bool, /* for ACC address */
    sdo2_high: bool, /* for GYRO address */
    range: AccRange,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    I2c(E),
    Unidentified
}

enum Device {
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
            range: AccRange::Range6G
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

    pub async fn read_acc(&mut self) -> Result<Acceleration, Error<E>> {
        let mut data = [0u8; 6];
        self.read_bytes(Device::Acc, regs::BMI088_ACC_X_LSB, &mut data).await?;
        let x = i16::from_le_bytes([data[0], data[1]]);
        let y = i16::from_le_bytes([data[2], data[3]]);
        let z = i16::from_le_bytes([data[4], data[5]]);
        Ok(Acceleration::new(x, y, z))
    }

    pub async fn init<D>(&mut self, delay: &mut D) -> Result<(), Error<E>> 
    where
        D: DelayNs
    {
        let byte = self.read_u8(Device::Acc, regs::BMI088_ACC_CHIP_ID).await?;
        #[cfg(feature = "defmt")]
        {
            defmt::trace!("Received {} for ACC_CHIP_ID, expecting 0x1e", byte);
        }
        if byte != 0x1e {
            return Err(Error::Unidentified);
        }

        // See section 3
        delay.delay_ms(1).await;
        self.write_u8(Device::Acc, regs::BMI088_ACC_PWR_CTRL, 0x04).await?;
        delay.delay_ms(50).await;
        
        Ok(())
    }

    pub async fn set_acc_range(&mut self, range: AccRange) -> Result<(), Error<E>> {
        self.write_u8(Device::Acc, regs::BMI088_ACC_CONF, range.into()).await?;
        self.range = range;
        Ok(())
    }

    pub fn acc_range(&self) -> AccRange {
        self.range
    }

    fn i2c_addr(&self, device: Device) -> u8 {
        match device {
            Device::Acc => if self.sdo1_high {
                regs::BMI088_ACC_ADDRESS_HIGH 
            } else {
                regs::BMI088_ACC_ADDRESS_LOW 
            },
            Device::Gyro => if self.sdo2_high {
                regs::BMI088_GYRO_ADDRESS_HIGH
            } else {
                regs::BMI088_GYRO_ADDRESS_LOW
            },
        }
    }

    async fn read_u8(&mut self, device: Device, reg: u8) -> Result<u8, Error<E>> {
        let mut out: [u8; 1] = [0; 1];
        self.i2c.write_read(self.i2c_addr(device), &[reg], &mut out).await.map_err(Error::I2c)?;
        Ok(out[0])
    }

    async fn read_bytes(&mut self, device: Device, reg: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c.write_read(self.i2c_addr(device), &[reg], buf).await.map_err(Error::I2c)
    }

    async fn write_u8(&mut self, device: Device, reg: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c.write(self.i2c_addr(device), &[reg, value]).await.map_err(Error::I2c)?;
        Ok(())
    }
}

