
use crate::constants::*;

/// The maximum number of iterations that the solver will do to find the drag
/// coefficient.
const SOLVER_MAX_ITERATIONS: usize = 30;
/// How small the derivative should be in a Newton-Raphson before we say it's
/// close enough.
const SOLVER_EPSILON: f32 = 1e-6;
/// How small the difference between consecutive guesses should be before we say
/// it's close enough.
const SOLVER_TOLERANCE: f32 = 1e-6;

/// A solver for the optimal drag coefficient of the rocket to get to a given
/// target altitude. This assumes that the rocket is operating within the
/// troposphere.
pub struct Solver {
    /// The target altitude of the rocket, in meters.
    target_m: f32,
}

impl Solver {

    /// Creates a new instance of the solver with a specified target altitude,
    /// in meters.
    pub fn new(target_m: f32) -> Self {
        Self {
            target_m
        }
    }    
    
    /// Solves for the optimal drag coefficient to reach the target altitude
    /// (see [`Self::target_altitude`]), given the current inertial
    /// characteristics of the rocket:
    /// - Altitude, in meters; and
    /// - The upward velocity of the rocket, in meters per second.
    pub fn solve(&self, altitude_m: f32, upward_velocity_mps: f32) -> f32 {
        // If we are outside of required operation parameters; assume the flaps
        // must be closed.
        if upward_velocity_mps > 210.0 || upward_velocity_mps < 0.0 {
            return ROCKET_CLOSE_CD;
        }

        let density = RHO_0_KGPM3 * libm::powf(
            1.0 + L_0_KPM * altitude_m / T_0_K,
            -BARO_EXP - 1.0
        );

        let mut curr_cd = ROCKET_CLOSE_CD + 0.2;
        
        for _ in 0..SOLVER_MAX_ITERATIONS {
            // Constant portion of the drag force.
            let k = f32::max(
                0.5 * density * curr_cd * ROCKET_CROSS_SECTIONAL_AREA_M2,
                1.0e-9
            );

            let v2 = upward_velocity_mps * upward_velocity_mps;
            let mg = ROCKET_BURNOUT_MASS_KG * GRAVITY_MPS2;
            
            let term1 = (k * v2 / mg) + 1.0;
            let term2 = libm::logf(term1);
            
            // Apogee prediction and predicted error
            let predicted_altitude = altitude_m + (ROCKET_BURNOUT_MASS_KG / (2.0 * k)) * term1;
            let residual = predicted_altitude - self.target_altitude();

            // Derivative of altitude over constant k
            let dalt_dk = -ROCKET_BURNOUT_MASS_KG * term1 / (2.0 * k * k) +
                (ROCKET_BURNOUT_MASS_KG / (2.0 * k)) * (v2 / mg) / term2;
                
            // Derivative of altitude over C_D (chain rule)
            let dk_dcd = 0.5 * density * ROCKET_CROSS_SECTIONAL_AREA_M2;
            let dalt_dcd = dalt_dk * dk_dcd;

            // If we are close enough, stop iterating.
            if dalt_dcd.abs() < SOLVER_EPSILON { break }

            let new_cd = f32::clamp(
                curr_cd - (residual / dalt_dcd),
                ROCKET_CLOSE_CD,
                ROCKET_OPEN_CD
            );

            // If the guesses have stalled out, stop iterating.
            if (curr_cd - new_cd).abs() < SOLVER_TOLERANCE {
                curr_cd = new_cd;
                break;
            }

            curr_cd = new_cd;
        }
        
        curr_cd
    }
    
    /// Returns the target altitude of the rocket, in meters.
    pub fn target_altitude(&self) -> f32 {
        self.target_m
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test when the rocket is going very fast compared to the target; and thus
    /// the flaps will remain entirely open during operation.
    #[test]
    fn too_fast() {
        // 10k feet per Vandal target
        let solver = Solver::new(3048.0);
        let cd = solver.solve(2000.0, 200.0);

        // Make sure we are opening here.
        assert_ne!(cd, ROCKET_CLOSE_CD);
        assert_eq!(cd, ROCKET_OPEN_CD);
    }

    /// Test when the rocket is going too slow, and will undershoot target.
    /// Expected to close the flaps.
    #[test]
    fn too_slow() {
        // 10k feet per Vandal target
        let solver = Solver::new(3048.0);
        let cd = solver.solve(2000.0, 15.0);
        
        // Make sure we are closing here.
        assert_ne!(cd, ROCKET_OPEN_CD);
        assert_eq!(cd, ROCKET_CLOSE_CD);
    }
}
