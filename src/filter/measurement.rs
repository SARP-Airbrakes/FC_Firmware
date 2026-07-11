use crate::bmi088::Bmi088Acceleration;


pub enum Measurement {
    ACC(Bmi088Acceleration)
}
