#![no_std]

mod measurements;
mod regs;

use core::array::TryFromSliceError;

use embedded_hal::i2c::{I2c, SevenBitAddress};

pub use measurements::*;

pub struct Bmp390<I> {
    i2c: I,
    sdo_high: bool
}


#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    I2c(E),
}

impl<I, E> Bmp390<I>
where
    I: I2c<SevenBitAddress, Error = E>
{

    pub fn new(i2c: I) -> Self {
        Self {
            i2c,
            sdo_high: false
        }
    }

    pub fn with_sdo_high(mut self) -> Self {
        self.sdo_high = true;
        self
    }

    pub fn read_coefficients(&mut self) -> Result<Coefficients, Error<E>> {
        let mut coeff = [0u8; 21];
        self.read_bytes(regs::BMP390_NVM_PAR_T1, &mut coeff)
            .map_err(Error::I2c)?;

        let t1 = u16::from_le_bytes(coeff[0..2].try_into().unwrap_or_default());
        let t2 = u16::from_le_bytes(coeff[2..4].try_into().unwrap_or_default());
        let t3 = coeff[4].cast_signed();
        
        let p1 = i16::from_le_bytes(coeff[5..7].try_into().unwrap_or_default());
        let p2 = i16::from_le_bytes(coeff[7..9].try_into().unwrap_or_default());
        let p3 = coeff[9].cast_signed();
        let p4 = coeff[10].cast_signed();
        let p5 = u16::from_le_bytes(coeff[11..13].try_into().unwrap_or_default());
        let p6 = u16::from_le_bytes(coeff[13..15].try_into().unwrap_or_default());
        let p7 = coeff[15].cast_signed();
        let p8 = coeff[16].cast_signed();
        let p9 = i16::from_le_bytes(coeff[17..19].try_into().unwrap_or_default());
        let p10 = coeff[19].cast_signed();
        let p11 = coeff[20].cast_signed();

        Ok(Coefficients {
            t1: f32::from(t1) * (2 << 8) as f32,
            t2: f32::from(t2) / (2 << 30) as f32,
            t3: f32::from(t3) /  562949953421312f32,
            p1: (f32::from(p1) - (2 << 14) as f32) / (2 << 20) as f32,
            p2: (f32::from(p2) - (2 << 14) as f32) / (2 << 29) as f32,
            p3: f32::from(p3) / 8589934592f32,
            p4: f32::from(p4) / 274877906944f32,
            p5: f32::from(p5) * (2 << 3) as f32,
            p6: f32::from(p6) / (2 << 6) as f32,
            p7: f32::from(p7) / (2 << 8) as f32,
            p8: f32::from(p8) / (2 << 15) as f32,
            p9: f32::from(p9) / 562949953421312f32,
            p10: f32::from(p10) / 562949953421312f32,
            p11: f32::from(p11) / 36893488147419103232f32,
        })
    }

    fn i2c_addr(&self) -> u8 {
        if self.sdo_high { regs::BMP390_ADDRESS_HIGH } else { regs::BMP390_ADDRESS_LOW }
    }

    fn read_u8(&mut self, reg: u8) -> Result<u8, E> {
        let mut out: [u8; 1] = [0; 1];
        self.i2c.write_read(self.i2c_addr(), &[reg], &mut out)?;
        Ok(out[0])
    }

    fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(self.i2c_addr(), &[reg], buf)
    }

    fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(self.i2c_addr(), &[reg, value])?;
        Ok(())
    }
}