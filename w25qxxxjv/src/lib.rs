#![no_std]

use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiBus};

mod regs;

pub use regs::Model;
use regs::StatusRegister;

pub struct W25qxxxjv<'a, S, CS, D> {
    spi: S,
    cs: CS,
    model: Model,
    delay: &'a mut D
}

pub type Wusize = u32; // Largest model has a 24-bit address space.

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<SE, PE> {
    Spi(SE),
    Pin(PE),
    ModelMismatch,
    OutOfBounds,
}

impl<'a, S, CS, D, SE, PE> W25qxxxjv<'a, S, CS, D> 
where
    S: SpiBus<Error = SE>,
    CS: OutputPin<Error = PE>,
    D: DelayNs
{
    pub fn new(spi: S, cs: CS, model: Model, delay: &'a mut D) -> Self {
        Self {
            spi,
            cs,
            model,
            delay
        }
    }

    pub fn destroy(self) -> S {
        self.spi
    }

    pub async fn init(&mut self) -> Result<(), Error<SE, PE>> {
        self.set_cs_high()?;
        self.delay.delay_ms(5).await; // allow CS to settle
        
        let (mfr_id, dev_id) = self.read_manufacturer_device_id().await?;
        if mfr_id != regs::W25QXXXJV_MANUFACTURER_ID || dev_id != self.model.device_id() {
            return Err(Error::ModelMismatch)
        }
        Ok(())
    }

    fn set_cs_high(&mut self) -> Result<(), Error<SE, PE>> {
        self.cs.set_high().map_err(Error::Pin)
    }

    fn set_cs_low(&mut self) -> Result<(), Error<SE, PE>> {
        self.cs.set_low().map_err(Error::Pin)
    }

    pub fn in_bounds(&self, addr: Wusize) -> bool {
        addr < self.model.capacity()
    }

    fn slice_in_bounds(&self, addr: Wusize, len: usize) -> bool {
        self.in_bounds(addr) && self.in_bounds(addr + len as Wusize - 1)
    }

    pub async fn read_data(&mut self, addr: Wusize, buf: &mut [u8]) -> Result<(), Error<SE, PE>> {
        if !self.slice_in_bounds(addr, buf.len()) {
            return Err(Error::OutOfBounds);
        }

        self.wait_until_not_busy().await?;

        let bytes = addr.to_be_bytes();
        self.set_cs_low()?;
        self.write(&[regs::W25QXXXJV_READ_DATA, bytes[1], bytes[2], bytes[3]]).await?;
        self.read(buf).await?;
        self.set_cs_high()
    }

    /// Writes data to one page (256-byte area of memory, which itself is 256-byte aligned).
    pub async fn page_program(&mut self, addr: Wusize, buf: &[u8]) -> Result<(), Error<SE, PE>> {
        if !self.slice_in_bounds(addr, buf.len()) {
            return Err(Error::OutOfBounds);
        }

        self.wait_until_not_busy().await?;
        self.write_enable().await?;

        let bytes = addr.to_be_bytes();
        self.set_cs_low()?;
        self.write(&[regs::W25QXXXJV_PAGE_PROGRAM, bytes[1], bytes[2], bytes[3]]).await?;
        self.write(buf).await?;
        self.set_cs_high()
    }

    /// Writes data to the chip larger than one page via sequential page programs.
    pub async fn write_data(&mut self, mut addr: Wusize, buf: &[u8]) -> Result<(), Error<SE, PE>> {
        if !self.slice_in_bounds(addr, buf.len()) {
            return Err(Error::OutOfBounds);
        }

        let mut buf_index = 0usize;

        while buf_index < buf.len() {
            // amount remaining in page
            let remaining = u32::min(0x100 - (addr & 0xff), (buf.len() - buf_index) as u32);

            #[cfg(feature = "defmt")]
            {
                defmt::trace!("mem write: addr: {:x} remaining: {}", addr, remaining);
            }

            self.page_program(addr, &buf[buf_index..buf_index + remaining as usize]).await?;
            addr += remaining;
            buf_index += remaining as usize;
        }
        
        Ok(())
    }

    pub async fn read_manufacturer_device_id(&mut self) -> Result<(u8, u8), Error<SE, PE>> {
        let mut out = [0u8; 2]; // manufacturer id, device id

        self.wait_until_not_busy().await?;

        self.set_cs_low()?;
        self.write(&[regs::W25QXXXJV_MANUFACTURER_DEVICE_ID, 0x00, 0x00, 0x00]).await?;
        self.read(&mut out).await?;
        self.set_cs_high()?;
        Ok(out.into())
    }

    /// Erases a sector (a 4kb-block).
    pub async fn erase_sector(&mut self, addr: Wusize) -> Result<(), Error<SE, PE>> {
        // ensure the address is aligned to the sectors
        let addr = addr & !0x0fff;
        let bytes = addr.to_be_bytes();

        self.wait_until_not_busy().await?;
        self.write_enable().await?;

        self.set_cs_low()?;
        self.write(&[regs::W25QXXXJV_SECTOR_ERASE, bytes[1], bytes[2], bytes[3]]).await?;
        self.set_cs_high()?;

        self.wait_until_not_busy().await?;

        Ok(())
    }

    /// Erases the entire chip. Takes up to a minute.
    pub async fn erase_chip(&mut self) -> Result<(), Error<SE, PE>> {
        self.wait_until_not_busy().await?;
        self.write_enable().await?;
        
        self.set_cs_low()?;
        self.write(&[regs::W25QXXXJV_CHIP_ERASE]).await?;
        self.set_cs_high()?;

        self.wait_until_not_busy().await?;
    
        Ok(())
    }

    pub async fn wait_until_not_busy(&mut self) -> Result<(), Error<SE, PE>> {
        while self.read_busy().await? {
            self.delay.delay_ms(10).await;
        }
        Ok(())
    }

    async fn write_enable(&mut self) -> Result<(), Error<SE, PE>> {
        self.set_cs_low()?;
        self.write(&[regs::W25QXXXJV_WRITE_ENABLE]).await?;
        self.set_cs_high()?;
        Ok(())
    }

    async fn read_busy(&mut self) -> Result<bool, Error<SE, PE>> {
        let sr1 = self.read_status_register(StatusRegister::SR1).await?;
        Ok((sr1 & 0x01) != 0x00)
    }

    async fn read_status_register(&mut self, reg: StatusRegister) -> Result<u8, Error<SE, PE>> {
        let mut out = 0u8;
        self.set_cs_low()?;
        self.write(&[reg.byte_read()]).await?;
        self.read(core::slice::from_mut(&mut out)).await?;
        self.set_cs_high()?;
        Ok(out)
    }

    async fn read(&mut self, buf: &mut [u8]) -> Result<(), Error<SE, PE>> {
        self.spi.read(buf).await.map_err(Error::Spi)
    }

    async fn write(&mut self, buf: &[u8]) -> Result<(), Error<SE, PE>> {
        self.spi.write(buf).await.map_err(Error::Spi)
    }
}