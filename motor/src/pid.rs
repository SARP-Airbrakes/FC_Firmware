
/// A wrapper around a motor controller that controls the position of the motor
/// (angle) using a PID controller.
pub struct Pid<H> {
    proportional: f32,
    integral: f32,
    derivative: f32,

    integral_error: f32,
    last_error: f32,
}

impl Pid {

}


