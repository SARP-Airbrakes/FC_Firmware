
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
