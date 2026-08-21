
pub const W25QXXXJV_MANUFACTURER_ID: u8 = 0xef;

pub const W25QXXXJV_PAGE_PROGRAM: u8 = 0x02;
pub const W25QXXXJV_READ_DATA: u8 = 0x03;
pub const W25QXXXJV_WRITE_ENABLE: u8 = 0x06;
pub const W25QXXXJV_SECTOR_ERASE: u8 = 0x20;
pub const W25QXXXJV_MANUFACTURER_DEVICE_ID: u8 = 0x90;

pub enum Model {
    W25q16jv,
    W25q32jv,
    W25q64jv,
    W25q128jv,
}

impl Model {
    pub const fn capacity(&self) -> crate::Wusize {
        match self {
            Model::W25q16jv => 1 << 21,
            Model::W25q32jv => 1 << 22,
            Model::W25q64jv => 1 << 23,
            Model::W25q128jv => 1 << 24,
        }
    }

    pub const fn device_id(&self) -> u8 {
        match self {
            Model::W25q16jv => 0x14,
            Model::W25q32jv => 0x15,
            Model::W25q64jv => 0x16,
            Model::W25q128jv => 0x17,
        }
    }
}

pub(crate) enum StatusRegister {
    SR1,
    SR2,
    SR3
}

impl StatusRegister {
    pub const fn byte_read(&self) -> u8 {
        match self {
            StatusRegister::SR1 => 0x05,
            StatusRegister::SR2 => 0x35,
            StatusRegister::SR3 => 0x15,
        }
    }
}