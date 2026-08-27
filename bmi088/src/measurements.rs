
const GRAVITY_EARTH: f32 = 9.81;

#[derive(Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Acceleration {
    x: i16,
    y: i16,
    z: i16,
}

pub trait AccelerationLike {
    fn x_raw(&self) -> i16;
    fn y_raw(&self) -> i16;
    fn z_raw(&self) -> i16;

    /// Calculates x-axis acceleration in m/s^2.
    fn x_ms2(&self, range: crate::AccRange) -> f32 {
        let x = self.x_raw() as f32;
        (x * GRAVITY_EARTH * Into::<f32>::into(range)) / 32768.0
    }

    /// Calculates y-axis acceleration in m/s^2.
    fn y_ms2(&self, range: crate::AccRange) -> f32 {
        let y = self.y_raw() as f32;
        (y * GRAVITY_EARTH * Into::<f32>::into(range)) / 32768.0
    }

    /// Calculates z-axis acceleration in m/s^2.
    fn z_ms2(&self, range: crate::AccRange) -> f32 {
        let z = self.z_raw() as f32;
        (z * GRAVITY_EARTH * Into::<f32>::into(range)) / 32768.0
    }

    /// Calculates the x-axis acceleration in milli-gravity units (thousandths
    /// of Earth's gravity).
    fn x_mg(&self, range: crate::AccRange) -> f32 {
        let x = self.x_raw() as f32;
        (x * 1000.0 * Into::<f32>::into(range)) / 32768.0
    }

    /// Calculates the y-axis acceleration in milli-gravity units (thousandths
    /// of Earth's gravity).
    fn y_mg(&self, range: crate::AccRange) -> f32 {
        let y = self.y_raw() as f32;
        (y * 1000.0 * Into::<f32>::into(range)) / 32768.0
    }

    /// Calculates the z-axis acceleration in milli-gravity units (thousandths
    /// of Earth's gravity).
    fn z_mg(&self, range: crate::AccRange) -> f32 {
        let z = self.z_raw() as f32;
        (z * 1000.0 * Into::<f32>::into(range)) / 32768.0
    }
}

impl Acceleration {

    pub(crate) fn new(x: i16, y: i16, z: i16) -> Self {
        Self { x, y, z }
    }
}

impl AccelerationLike for Acceleration {
    fn x_raw(&self) -> i16 { self.x }
    fn y_raw(&self) -> i16 { self.y }
    fn z_raw(&self) -> i16 { self.z }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccSelfTestResult {
    x: i16,
    y: i16,
    z: i16
}

impl AccSelfTestResult {
    
    pub(crate) fn new(positive: Acceleration, negative: Acceleration) -> Self {
        Self {
            x: positive.x - negative.x,
            y: positive.y - negative.y,
            z: positive.z - negative.z,
        }
    }
}

impl AccelerationLike for AccSelfTestResult {
    fn x_raw(&self) -> i16 { self.x }
    fn y_raw(&self) -> i16 { self.y }
    fn z_raw(&self) -> i16 { self.z }
}