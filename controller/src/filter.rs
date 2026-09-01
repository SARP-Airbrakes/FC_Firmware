//! This filter assumes no covariance between the accelerometer and barometer
//! measurements; obviously, this is false.
//! 
//! Important notation: x_k = x(t_k).

use nalgebra::{Matrix1, Matrix4, RowVector4, Vector4};

use crate::{stage::FlightStage, constants::*};

/// The constant coefficient that appearsawhen deriving the barometric equation.
const ALPHA: f32 = BARO_EXP * L_0_KPM * P_0_PA / T_0_K;

/// Minimum time between propagations.
const MIN_PROPAGATION_TIME: f32 = 1e-4;

/// The Jacobian of f(x(t),u(t)) over x(t).
const F_MATRIX: Matrix4<f32> = Matrix4::new(
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, -1.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
);

/// The measurement noise covariance matrix (a scalar variance) for barometer
/// pressure measurements.
const R_P: Matrix1<f32> = Matrix1::new(1.1); // TODO: measure
/// The measurement noise covariance matrix (a scalar variance) for
/// accelerometer acceleration measurements.
const R_A: Matrix1<f32> = Matrix1::new(1.8e-2); // TODO: estimate

/// The variance in the drift of the accelerometer bias.
const ACCELEROMETER_BIAS_NOISE_VARIANCE: f32 = 1.0e-4;
const BAROMETER_BIAS_NOISE_VARIANCE: f32 = 5.0e-3;

/// This class implements an Extended Kalman filter that tracks the altitude of
/// the rocket in flight.
pub struct Filter {
    /// x_k = [altitude, upward velocity, accelerometer bias (in m/s^2), barometer bias (in Pa)]^T
    state: Vector4<f32>,
    /// P_k
    covariance: Matrix4<f32>,
    /// t_{k-1}, or the last time that the state was propagated
    last_time_ms: u64,
    /// The last measured upward acceleration.
    last_upward_acceleration: Option<f32>,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Uninitialized,
    BackwardPropagation,
    NotEnoughData
}

impl Filter {

    pub fn new(altitude: f32, time_ms: u64) -> Self {
        Filter {
            state: Vector4::<f32>::x() * altitude,
            covariance: Matrix4::from_partial_diagonal(&[1.0, 0.0, 0.2, 0.5]),
            last_time_ms: time_ms,
            last_upward_acceleration: Some(0.0)
        }
    }

    pub fn propagate(&mut self, time_ms: u64, measured_upward_acceleration: f32) -> Result<(), Error> {
        let delta_t = (time_ms as i64 - self.last_time_ms as i64) as f32 / 1000.0;
        if delta_t < 0.0 { // prevent any backwards propagation
            return Err(Error::BackwardPropagation);
        }
        // Skip this propagation if the last one was recent.
        if delta_t < MIN_PROPAGATION_TIME { return Ok(()) }

        self.last_time_ms = time_ms;

        // Update state.
        {
            // State transition matrix.
            let a = Matrix4::<f32>::new(
                1.0, delta_t, -0.5 * delta_t * delta_t, 0.0,
                0.0, 1.0, -delta_t, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0
            );
            // Control matrix.
            let b = Vector4::<f32>::new(
                0.5 * delta_t * delta_t,
                delta_t,
                0.0,
                0.0
            );
            // See Self::upward_acceleration
            self.state = a * self.state + b * (measured_upward_acceleration - GRAVITY_MPS2);
            self.last_upward_acceleration = Some(measured_upward_acceleration);
        }

        // Update covariance.
        self.covariance = F_MATRIX * self.covariance * F_MATRIX.transpose() + self.q_matrix(delta_t);
        Ok(())
    }

    fn q_matrix(&self, delta_t: f32) -> Matrix4<f32> {
        let acc_var = R_A.to_scalar();
        let delta_t2 = delta_t * delta_t;
        let delta_t3 = delta_t2 * delta_t;
        let delta_t4 = delta_t2 * delta_t2;
        let delta_t5 = delta_t3 * delta_t2;
        // Discretization of the continuous noise process. R_A also represents
        // the noise in the control (or forcing) input that's why it's also here
        Matrix4::new(
            (delta_t5 * ACCELEROMETER_BIAS_NOISE_VARIANCE / 20.0) + (delta_t3 * acc_var / 3.0),
            (delta_t4 * ACCELEROMETER_BIAS_NOISE_VARIANCE / 8.0) + (delta_t2 * acc_var / 2.0),
            -delta_t3 * ACCELEROMETER_BIAS_NOISE_VARIANCE / 6.0,
            0.0,
            
            (delta_t4 * ACCELEROMETER_BIAS_NOISE_VARIANCE / 8.0) + (delta_t2 * acc_var / 2.0),
            (delta_t3 * ACCELEROMETER_BIAS_NOISE_VARIANCE / 3.0) + (delta_t * acc_var),
            -delta_t2 * ACCELEROMETER_BIAS_NOISE_VARIANCE / 2.0,
            0.0,

            -delta_t3 * ACCELEROMETER_BIAS_NOISE_VARIANCE / 6.0,
            -delta_t2 * ACCELEROMETER_BIAS_NOISE_VARIANCE / 2.0,
            delta_t * ACCELEROMETER_BIAS_NOISE_VARIANCE,
            0.0,

            0.0,
            0.0,
            0.0,
            delta_t * BAROMETER_BIAS_NOISE_VARIANCE,
        )
    }

    /// Updates the filter based on a pressure measurement from the barometer.
    pub fn update_pressure(&mut self, time_ms: u64, pressure_pa: f32) -> Result<(), Error> {
        // First, propagate. We use the old acceleration here, assuming only
        // small changes between when the IMU measurement is received and when
        // the barometer measurement is received.
        self.propagate(
            time_ms, 
            self.last_upward_acceleration.ok_or(Error::Uninitialized)?
        )?;

        let s = self.altitude();

        // The Jacobian of h(x_k) over x_k.
        let h = RowVector4::<f32>::new(
            -ALPHA * libm::powf((T_0_K + L_0_KPM * s) / T_0_K, -BARO_EXP - 1.0), 
            0.0,
            0.0, 
            0.0
        );

        // Calculated Kalman gain.
        let k = self.covariance * h.transpose() 
            / (h * self.covariance * h.transpose() + R_P).to_scalar();

        // The predicted pressure given our a priori state.
        let p_pred = P_0_PA * libm::powf((T_0_K + L_0_KPM * s) / T_0_K, -BARO_EXP);

        // Update our a priori state.
        self.state = self.state + k * (pressure_pa - p_pred);

        // Update our a priori covariance.
        self.covariance = (Matrix4::<f32>::identity() - k * h) * self.covariance;
        Ok(())
    }

    /// Updates the filter based on a received accelerometer measurement, the
    /// drag coefficient of the rocket, and the currently known flight stage (to
    /// estimate the current acceleration of the rocket).
    pub fn update_acceleration(
        &mut self, 
        time_ms: u64, 
        measured_upward_acceleration: f32, 
        drag_coefficient: f32, 
        flight_stage: FlightStage
    ) -> Result<(), Error> {
        // After apogee, the acceleration is too unpredictable.
        match flight_stage {
            FlightStage::Boost { .. } | // TODO: temporary, see TODO below
            FlightStage::Recovery => { return Err(Error::NotEnoughData); },
            _ => {}
        };

        // Propagate using the measured acceleration.
        self.propagate(time_ms, measured_upward_acceleration)?;

        // The predicted acceleration measurement.
        let prediction = self.upward_acceleration_bias() + match flight_stage {
            FlightStage::Idle => {
                // The rocket is on the ground here, so it is sensing the normal
                // force from the ground.
                GRAVITY_MPS2
            },
            FlightStage::Boost { start_time_ms: _ } => {
                // TODO: thrust profile
                -self.drag(drag_coefficient)
            },
            FlightStage::InactiveCoast |
            FlightStage::ActiveCoast => -self.drag(drag_coefficient),
            _ => unreachable!()
        };
        
        // The Jacobian of the accelerometer prediction
        let h = RowVector4::new(0.0, 0.0, 1.0, 0.0) + match flight_stage {
            FlightStage::Idle => RowVector4::zeros(),
            FlightStage::Boost { .. } |
            FlightStage::InactiveCoast |
            FlightStage::ActiveCoast => -self.drag_jacobian(drag_coefficient),
            _ => unreachable!()
        };

        // Kalman gain
        let k = self.covariance * h.transpose() 
            / (h * self.covariance * h.transpose() + R_A).to_scalar();

        // Update the state.
        self.state = self.state + k * (measured_upward_acceleration - prediction);

        // Update the covariance.
        self.covariance = (Matrix4::<f32>::identity() - k * h) * self.covariance;

        Ok(())
    }

    fn drag(&self, drag_coefficient: f32) -> f32 {
        let density = estimated_density(self.altitude());
        let vel = self.upward_velocity();
        0.5 * density * vel * vel * drag_coefficient * ROCKET_CROSS_SECTIONAL_AREA_M2
    }

    fn drag_jacobian(&self, drag_coefficient: f32) -> RowVector4<f32> {
        let density = estimated_density(self.altitude());
        let density_ds = RHO_0_KGPM3 * (-BARO_EXP - 1.0) * (L_0_KPM / T_0_K) * libm::powf(
            1.0 + L_0_KPM * self.altitude() / T_0_K,
            -BARO_EXP - 2.0
        );

        let vel = self.upward_velocity();
        RowVector4::new(
            0.5 * density_ds * vel * vel * drag_coefficient * ROCKET_CROSS_SECTIONAL_AREA_M2,
            density * vel * drag_coefficient * ROCKET_CROSS_SECTIONAL_AREA_M2,
            0.0,
            0.0
        )
    }

    /// Returns the filters estimation of the altitude, in meters.
    pub fn altitude(&self) -> f32 {
        self.state.x
    }

    /// Returns the filters estimation of the upward velocity, in meters per
    /// second.
    pub fn upward_velocity(&self) -> f32 {
        self.state.y
    }

    /// Returns the filters estimation of the bias of the accelerometer in
    /// measuring the upward acceleration, in meters per second squared.
    pub fn upward_acceleration_bias(&self) -> f32 {
        self.state.z
    }

    /// Returns the upward acceleration of the rocket, accounting for the
    /// accelerometer bias. This assumes that a fixed point on the Earth is in a
    /// inertial reference frame.
    pub fn upward_acceleration(&self, measured_upward_acceleration: f32) -> f32 {
        measured_upward_acceleration - GRAVITY_MPS2 - self.upward_acceleration_bias()
    }

    pub fn state(&self) -> Vector4<f32> {
        self.state
    }

    pub fn covariance(&self) -> Matrix4<f32> {
        self.covariance
    }
}


