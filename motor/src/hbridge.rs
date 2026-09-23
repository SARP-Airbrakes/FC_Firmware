
use embedded_hal::pwm::SetDutyCycle;

/// H-bridge controller error type.
pub enum Error<P> {
    Pwm(P),
}
 
/// Represents an H-bridge circuit that controls a motor, controlled by two
/// pins: IN1 and IN2.
pub struct Hbridge<P> {
    in_1: P,
    in_2: P,
    epsilon: f32,
}

impl<P, E> Hbridge<P>
where
    P: SetDutyCycle<Error = E>
{

    /// Creates a new H-bridge controller from the two pins.
    pub fn new(
        in_1: P,
        in_2: P,
    ) -> Self {
        Self {
            in_1,
            in_2,
            epsilon: 2e-3,
        }
    }
    
    /// Sets the epsilon of the H-bridge controller.
    ///
    /// The epsilon is a value that is used when setting the power of motor
    /// control (see [`Self::set_power`]), to delineate an area near zero that
    /// is used for braking. For braking to be disabled, set the epsilon to
    /// zero.
    pub fn with_epsilon(mut self, epsilon: f32) -> Self {
        self.epsilon = epsilon;
        self
    }

    /// Sets the duty cycle of the PWM controlling the H-bridge. 
    ///
    /// The given value is clamped to the range [0,1]. The sign of the cycle
    /// represents the direction of the current going through the H-bridge: 
    /// - When negative, IN1 is left off while IN2 is set to the given cycle.
    /// - When positive, IN2 is left off and IN1 is set to the given cycle.
    /// - When near zero (controlled by [`Self::with_epsilon`]), both are set
    ///   to 100% duty cycle (this is for braking).
    pub fn set_power(&mut self, cycle: f32) -> Result<(), Error<E>> {
        if cycle < -self.epsilon {
            self.in_1.set_duty_cycle_fully_off()?;
            self.in_2.set_duty_cycle((self.in_2.max_duty_cycle() as f32 * -cycle) as u16)?;
        } else if cycle > self.epsilon {
            self.in_1.set_duty_cycle((self.in_1.max_duty_cycle() as f32 * cycle) as u16)?;
            self.in_2.set_duty_cycle_fully_off()?;
        } else {
            self.in_1.set_duty_cycle_fully_on()?;
            self.in_2.set_duty_cycle_fully_on()?;
        }
        Ok(())
    }
}

impl<E> From<E> for Error<E> {
    fn from(value: E) -> Self {
        Error::Pwm(value)
    }
}
