
const GRAVITY_EARTH: f32 = 9.81;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Acceleration {
    x: i16,
    y: i16,
    z: i16,
}

impl Acceleration {

    pub fn new(x: i16, y: i16, z: i16) -> Self {
        Acceleration { x, y, z }
    }

    pub fn x_raw(&self) -> i16 {
        self.x
    }

    pub fn y_raw(&self) -> i16 {
        self.y
    }

    pub fn z_raw(&self) -> i16 {
        self.z
    }

    pub fn x_ms2(&self, range: crate::AccRange) -> f32 {
        let x = self.x_raw() as f32;
        (x * GRAVITY_EARTH * Into::<f32>::into(range)) / 32768.0
    }

    pub fn y_ms2(&self, range: crate::AccRange) -> f32 {
        let y = self.y_raw() as f32;
        (y * GRAVITY_EARTH * Into::<f32>::into(range)) / 32768.0
    }

    pub fn z_ms2(&self, range: crate::AccRange) -> f32 {
        let z = self.z_raw() as f32;
        (z * GRAVITY_EARTH * Into::<f32>::into(range)) / 32768.0
    }
}
