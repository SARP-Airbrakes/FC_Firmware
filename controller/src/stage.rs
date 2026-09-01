
// TODO
const MAXIMUM_ACTIVE_VELOCITY: f32 = 200.0;

/// Stages of flight.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FlightStage {
    /// Waiting on the pad.
    Idle,
    /// While the motor is burning.
    Boost { start_time_ms: u64 },
    /// After burnout, however too fast for deployment.
    InactiveCoast,
    /// After burnout, and while we are slow enough to deploy.
    ActiveCoast,
    /// After apogee.
    Recovery
}

impl FlightStage {
    
    /// Checks if the rocket should transition out of this stage of flight using
    /// known inertial characteristics of the rocket.
    /// 
    /// Importantly, this assumes that a fixed point on the Earth is an inertial
    /// reference frame; see [`crate::filter::Filter::upward_acceleration`].
    pub fn next(
        &self,
        time_ms: u64,
        upward_acceleration: f32, /* m/s^2 */
        upward_velocity: f32 /* m/s */
    ) -> Option<FlightStage> {
        match self {
            FlightStage::Idle => {
                // This transition should occur on launch.
                if upward_acceleration > 10.0 &&
                    upward_velocity > 5.0
                {
                    return Some(FlightStage::Boost { start_time_ms: time_ms })
                }
            }
            FlightStage::Boost { .. } => {
                // This transition should occur on burnout.
                if upward_acceleration < -12.0 { // gravity + some drag
                    return Some(FlightStage::InactiveCoast)
                }
            },
            FlightStage::InactiveCoast { .. } => {
                // This transition should occur when the velocity goes under the
                // deployment maximum (Mach 0.7).
                if upward_velocity < MAXIMUM_ACTIVE_VELOCITY {
                    return Some(FlightStage::ActiveCoast)
                }
            },
            FlightStage::ActiveCoast { .. } => {
                // This transition should occur at apogee.
                if upward_velocity < -0.0 {
                    return Some(FlightStage::Recovery);
                }
            },
            _ => {}
        }

        None
    }
}