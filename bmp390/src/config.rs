
use core::ops::{BitAnd, BitOr, Not};

/// Mode of operation. See Section 4.3.17.
pub enum PowerCtrlMode {
    Sleep,
    Forced,
    Normal
}

/// PWR_CTLR register values. See Section 4.3.17.
pub enum PowerCtrl {
    PressureEnable,
    TemperatureEnable,
    Mode(PowerCtrlMode),
}

// Arithmetic implementations for [`PowerCtrl`].
impl From<PowerCtrl> for u8 {
    fn from(value: PowerCtrl) -> Self {
        match value {
            PowerCtrl::PressureEnable => 0x01,
            PowerCtrl::TemperatureEnable => 0x02,
            PowerCtrl::Mode(m) => match m {
                PowerCtrlMode::Sleep => 0x00,
                PowerCtrlMode::Forced => 0x10,
                PowerCtrlMode::Normal => 0x30,
            }
        }
    }
}

impl BitOr for PowerCtrl {
    type Output = u8;
    
    fn bitor(self, rhs: Self) -> Self::Output {
        u8::from(self) | u8::from(rhs)
    }
}

impl BitOr<PowerCtrl> for u8 {
    type Output = u8;

    fn bitor(self, rhs: PowerCtrl) -> Self::Output {
        self | u8::from(rhs)
    }
}

impl BitAnd for PowerCtrl {
    type Output = u8;

    fn bitand(self, rhs: Self) -> Self::Output {
        u8::from(self) & u8::from(rhs)
    }
}

impl BitAnd<PowerCtrl> for u8 {
    type Output = u8;

    fn bitand(self, rhs: PowerCtrl) -> Self::Output {
        self & u8::from(rhs)
    }
}

impl Not for PowerCtrl {
    type Output = u8;

    fn not(self) -> Self::Output {
        !u8::from(self)
    }
}