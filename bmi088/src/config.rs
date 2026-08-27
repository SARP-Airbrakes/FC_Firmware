
macro_rules! implement_bit_or {
    ($x:ident) => {
        impl ::core::ops::BitOr for $x {
            type Output = u8;

            fn bitor(self, rhs: Self) -> Self::Output {
                u8::from(self) | u8::from(rhs)
            }
        }

        impl ::core::ops::BitOr<u8> for $x {
            type Output = u8;
            
            fn bitor(self, rhs: u8) -> Self::Output {
                u8::from(self) | rhs
            }
        }

        impl ::core::ops::BitOr<$x> for u8 {
            type Output = Self;

            fn bitor(self, rhs: $x) -> Self::Output {
                self | u8::from(rhs)
            }
        }
    }
}

macro_rules! implement_bit_ops {
    ($x:ident) => {
        impl ::core::ops::Not for $x {
            type Output = u8;

            fn not(self) -> Self::Output {
                !u8::from(self)
            }
        }

        impl ::core::ops::BitAnd for $x {
            type Output = u8;

            fn bitand(self, rhs: Self) -> Self::Output {
                u8::from(self) & u8::from(rhs)
            }
        }

        impl ::core::ops::BitAnd<u8> for $x {
            type Output = u8;
            
            fn bitand(self, rhs: u8) -> Self::Output {
                u8::from(self) & rhs
            }
        }

        impl ::core::ops::BitAnd<$x> for u8 {
            type Output = Self;

            fn bitand(self, rhs: $x) -> Self::Output {
                self & u8::from(rhs)
            }
        }

        implement_bit_or!($x);
    }
}

#[derive(Default, Clone, Copy)]
pub enum AccRange {
    Range3G,
    #[default]
    Range6G,
    Range12G,
    Range24G,
}

impl From<AccRange> for f32 {
    fn from(value: AccRange) -> Self {
        match value {
            AccRange::Range3G => 3.0,
            AccRange::Range6G => 6.0,
            AccRange::Range12G => 12.0,
            AccRange::Range24G => 24.0,
        }
    }
}

impl From<AccRange> for u8 {
    fn from(value: AccRange) -> Self {
        match value {
            AccRange::Range3G => 0x00,
            AccRange::Range6G => 0x01,
            AccRange::Range12G => 0x02,
            AccRange::Range24G => 0x03,
        }
    }
}

/// Accelerometer output data rate.
#[derive(Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccOdr {
    /// 12.5 Hz
    Hz12_5,
    /// 25 Hz
    Hz25,
    /// 50 Hz
    Hz50,
    /// 100 Hz
    #[default]
    Hz100,
    /// 200 Hz
    Hz200,
    /// 400 Hz
    Hz400,
    /// 800 Hz
    Hz800,
    /// 1600 Hz
    Hz1600,
}

impl From<AccOdr> for u8 {
    fn from(value: AccOdr) -> Self {
        match value {
            AccOdr::Hz12_5 => 0x05,
            AccOdr::Hz25 => 0x06,
            AccOdr::Hz50 => 0x07,
            AccOdr::Hz100 => 0x08,
            AccOdr::Hz200 => 0x09,
            AccOdr::Hz400 => 0x0a,
            AccOdr::Hz800 => 0x0b,
            AccOdr::Hz1600 => 0x0c,
        }
    }
}

/// Oversample rate for the accelerometer (regarding the low-pass filter).
#[derive(Default, Clone, Copy)]
pub enum AccOsr {
    /// Normal (average 4) mode.
    #[default]
    Normal,
    /// 2-fold oversampling.
    Osr2,
    /// 4-fold oversampling.
    Osr4,
}

impl From<AccOsr> for u8 {
    fn from(value: AccOsr) -> Self {
        match value {
            AccOsr::Normal => 0x02,
            AccOsr::Osr2 => 0x01,
            AccOsr::Osr4 => 0x00,
        }
    }
}

pub enum AccConf {
    /// Output data rate.
    Odr(AccOdr),
    /// Oversampling rate.
    Osr(AccOsr),
}

impl From<AccConf> for u8 {
    fn from(value: AccConf) -> Self {
        match value {
            AccConf::Odr(odr) => odr.into(),
            AccConf::Osr(osr) => u8::from(osr) << 4,
        }
    }
}

implement_bit_or!(AccConf);

pub enum AccIntConf {
    EnableInput,
    EnableOutput,
    PushPull,
    OpenDrain,
    ActiveLow,
    ActiveHigh,
}

impl From<AccIntConf> for u8 {
    fn from(value: AccIntConf) -> Self {
        match value {
            AccIntConf::EnableInput => 0x10,
            AccIntConf::EnableOutput => 0x08,
            AccIntConf::OpenDrain => 0x04,
            AccIntConf::ActiveHigh => 0x02,
            AccIntConf::PushPull |
            AccIntConf::ActiveLow => 0x00,
        }
    }
}

implement_bit_ops!(AccIntConf);

pub enum AccIntMap {
    /// Map data ready interrupt to INT1
    Int1Drdy,
    /// Map data ready interrupt to INT2
    Int2Drdy,
}

impl From<AccIntMap> for u8 {
    fn from(value: AccIntMap) -> Self {
        match value {
            AccIntMap::Int1Drdy => 0x04,
            AccIntMap::Int2Drdy => 0x40,
        }
    }
}

implement_bit_ops!(AccIntMap);

pub(crate) enum AccSelfTest {
    Disabled,
    Positive,
    Negative,
}

impl From<AccSelfTest> for u8 {
    fn from(value: AccSelfTest) -> Self {
        match value {
            AccSelfTest::Disabled => 0x00,
            AccSelfTest::Positive => 0x0D,
            AccSelfTest::Negative => 0x09,
        }
    }
}