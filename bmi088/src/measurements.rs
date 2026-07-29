
pub struct Bmi088Acceleration {
    x: i16,
    y: i16,
    z: i16,
}

impl Bmi088Acceleration {

    pub fn new(x: i16, y: i16, z: i16) -> Self {
        Bmi088Acceleration { x, y, z }
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


}
