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

    pub fn set_cs_high(&mut self) -> Result<(), Error<SE, PE>> {
        self.cs.set_high().map_err(Error::Pin)
    }

    pub fn set_cs_low(&mut self) -> Result<(), Error<SE, PE>> {
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
    pub async fn write_data(&mut self, addr: Wusize, buf: &[u8]) -> Result<(), Error<SE, PE>> {
        if !self.slice_in_bounds(addr, buf.len()) {
            return Err(Error::OutOfBounds);
        }
        let buf_len: u32 = buf.len() as u32; // proven smaller than u32
        let mut write_index = 0u32;
        let page_pos = addr & 0xff;
        let remaining = 0xff - page_pos;
        
        write_index += u32::min(remaining, buf_len);
        self.page_program(addr, &buf[..write_index as usize]).await?;

        // after this, page_pos should always be 0

        while write_index < buf.len() as u32 {
            let old_write_index = write_index;
            write_index += u32::min(0xff, buf_len - write_index);
            self.page_program(addr, &buf[old_write_index as usize..write_index as usize]).await?;
        }

        Ok(())
    }

    pub async fn read_manufacturer_device_id(&mut self) -> Result<(u8, u8), Error<SE, PE>> {
        let mut out = [0u8; 2]; // manufacturer id, device id
        self.set_cs_low()?;
        self.write(&[regs::W25QXXXJV_MANUFACTURER_DEVICE_ID, 0x00, 0x00, 0x00]).await?;
        self.read(&mut out).await?;
        self.set_cs_high()?;
        Ok(out.into())
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