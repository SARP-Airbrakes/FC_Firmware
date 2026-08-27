#![no_std]

use embedded_hal_async::{delay::DelayNs, i2c::{I2c, SevenBitAddress}};

mod regs;
mod config;
mod measurements;

pub use measurements::*;
pub use config::*;

pub const ACC_CHIP_ID: u8 = 0x1e;

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
    Wait
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
            range: AccRange::default(),
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

    /// Reads the chip id from the accelerometer. Expected: 0x1e.
    pub async fn read_acc_chip_id(&mut self) -> Result<u8, Error<E>> {
        self.read_u8(Device::Acc, regs::BMI088_ACC_CHIP_ID).await
    }

    pub async fn enable_acc(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<E>> {
        // See section 4.1.1; 50 ms is used to ensure settled sensor (see section 3)
        delay.delay_ms(1).await;
        self.write_u8(Device::Acc, regs::BMI088_ACC_PWR_CTRL, 0x04).await?;
        delay.delay_ms(50).await;
        Ok(())
    }

    pub async fn disable_acc(&mut self) -> Result<(), Error<E>> {
        self.write_u8(Device::Acc, regs::BMI088_ACC_PWR_CTRL, 0x00).await
    }

    pub async fn reset_acc(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<E>> {
        self.write_u8(Device::Acc, regs::BMI088_ACC_SOFTRESET, 0xb6).await?;
        delay.delay_ms(1).await;
        self.range = AccRange::default();
        Ok(())
    }

    pub async fn set_acc_range(&mut self, range: AccRange) -> Result<(), Error<E>> {
        self.write_u8(Device::Acc, regs::BMI088_ACC_RANGE, range.into()).await?;
        self.range = range;
        Ok(())
    }

    pub fn acc_range(&self) -> AccRange {
        self.range
    }

    /// Runs through recommended self-test procedure for the accelerometer (see
    /// section 4.6.1). Warning: this also resets the device, requiring
    /// reconfiguration.
    pub async fn self_test_acc(&mut self, delay: &mut impl DelayNs) -> Result<AccSelfTestResult, Error<E>> {
        // Configure
        self.set_acc_range(AccRange::Range24G).await?;
        self.set_acc_conf(AccConf::Osr(AccOsr::Normal) | AccConf::Odr(AccOdr::Hz1600)).await?;

        // Positive polarity
        delay.delay_ms(2).await;
        self.set_self_test(AccSelfTest::Positive).await?;
        delay.delay_ms(50).await;

        let positive = self.read_acc().await?;

        // Negative polarity
        self.set_self_test(AccSelfTest::Negative).await?;
        delay.delay_ms(50).await;

        let negative = self.read_acc().await?;

        // Disable
        self.set_self_test(AccSelfTest::Disabled).await?;
        delay.delay_ms(50).await;

        // Reset and return
        self.reset_acc(delay).await?;
        Ok(AccSelfTestResult::new(positive, negative))
    }

    async fn set_self_test(&mut self, conf: AccSelfTest) -> Result<(), Error<E>> {
        self.write_u8(Device::Acc, regs::BMI088_ACC_SELF_TEST, conf.into()).await
    }

    /// Sets the configuration of the accelerometer (see [`AccConf`])
    pub async fn set_acc_conf(&mut self, conf: impl Into<u8>) -> Result<(), Error<E>> {
        // 7th bit must always be one.
        self.write_u8(Device::Acc, regs::BMI088_ACC_CONF, conf.into() | 0x80).await
    }

    /// Configures the INT1 pin with the given configuration (see [`AccIntConf`]).
    pub async fn set_int1_conf(&mut self, conf: impl Into<u8>) -> Result<(), Error<E>> {
        self.write_u8(Device::Acc, regs::BMI088_INT1_IO_CONF, conf.into()).await
    }

    /// Configures the INT2 pin with the given configuration (see [`AccIntConf`]).
    pub async fn set_int2_conf(&mut self, conf: impl Into<u8>) -> Result<(), Error<E>> {
        self.write_u8(Device::Acc, regs::BMI088_INT2_IO_CONF, conf.into()).await
    }

    /// Configures how the accelerometer interrupts are mapped (see [`AccIntMap`]).
    pub async fn set_int1_int2_map(&mut self, conf: impl Into<u8>) -> Result<(), Error<E>> {
        self.write_u8(Device::Acc, regs::BMI088_INT1_INT2_MAP_DATA, conf.into()).await
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

