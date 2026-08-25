
#[derive(Clone, Copy)]
pub enum AccRange {
    Range3G,
    Range6G,
    Range12G,
    Range24G
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