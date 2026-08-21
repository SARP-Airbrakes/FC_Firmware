
/// Specific gas constant for dry air (J kg^-1 K^-1)
const R_SPECIFIC_DRY_AIR: f32 = 287.052874;
/// Acceleration from gravity at sea level (m s^-2).
const GRAVITY: f32 = 9.80665;
/// Pressure at sea level.
const SEA_LEVEL_PRESSURE: f32 = 101325.0;
const ALPHA: f32 = R_SPECIFIC_DRY_AIR / GRAVITY;

/// Calibration coefficients read from the sensor.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Coefficients {
    // Temperature coefficients
    pub(crate) t1: f32,
    pub(crate) t2: f32,
    pub(crate) t3: f32,

    // Pressure coefficients
    pub(crate) p1: f32,
    pub(crate) p2: f32,
    pub(crate) p3: f32,
    pub(crate) p4: f32,
    pub(crate) p5: f32,
    pub(crate) p6: f32,
    pub(crate) p7: f32,
    pub(crate) p8: f32,
    pub(crate) p9: f32,
    pub(crate) p10: f32,
    pub(crate) p11: f32,
}

/// Pressure reading from the sensor.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Pressure([u8; 3]); // uncompensated: XLSB, LSB, MSB

impl Pressure {
    pub fn new(data: [u8; 3]) -> Self {
        Self(data)
    }

    /// Using the calibration coefficients and a temperature, calculates the
    /// pressure reading in Pa from the uncompensated data.
    pub fn compensate(self, calib: &Coefficients, temp: f32) -> f32 {
        // See Appendix 0, section 8.6.
        let uncomp = u32::from(self) as f32;

        let partial_data1 = calib.p6 * temp;
        let partial_data2 = calib.p7 * (temp * temp);
        let partial_data3 = calib.p8 * (temp * temp * temp);
        let partial_out1 = calib.p5 + partial_data1 + partial_data2 + partial_data3;

        let partial_data1 = calib.p2 * temp;
        let partial_data2 = calib.p3 * (temp * temp);
        let partial_data3 = calib.p4 * (temp * temp * temp);
        let partial_out2 = uncomp * 
            (calib.p1 + partial_data1 + partial_data2 + partial_data3);
        
        let partial_data1 = uncomp * uncomp;
        let partial_data2 = calib.p9 + calib.p10 * temp;
        let partial_data3 = partial_data1 * partial_data2 + 
            (uncomp * uncomp * uncomp) * calib.p11;

        partial_out1 + partial_out2 + partial_data3
    }

    /// Considering the pressure measurement to be an accurate reflection of
    /// the outside air pressure, estimates the altitude (in meters) in the
    /// troposphere that corresponds to this pressure.
    pub fn estimate_altitude_hypsometric(press: f32, temp: f32) -> f32 {
        ALPHA * temp * libm::logf(SEA_LEVEL_PRESSURE / press)
    }
}

impl From<Pressure> for u32 {
    fn from(value: Pressure) -> Self {
        u32::from_le_bytes([value.0[0], value.0[1], value.0[2], 0x00])
    }
}

/// Temperature reading from the sensor.
pub struct Temperature([u8; 3]); // uncompensated: MSB, LSB, XLSB

impl Temperature {
    pub fn new(data: [u8; 3]) -> Self {
        Self(data)
    }

    /// Using the given calibration coefficients, calculates the temperature in
    /// Celsius from this measurement.
    pub fn compensate(self, calib: &Coefficients) -> f32 {
        let uncomp = u32::from(self) as f32;
        let partial_data1 = uncomp - calib.t1;
        let partial_data2 = partial_data1 * calib.t2;
        partial_data2 + (partial_data1 * partial_data1) * calib.t3
    }
}

impl From<Temperature> for u32 {
    fn from(value: Temperature) -> Self {
        u32::from_le_bytes([value.0[0], value.0[1], value.0[2], 0x00])
    }
}