//! This module contains physical constants as well as assorted design
//! parameters of the rocket and airbrakes system.

/// The cross sectional area of the rocket (m^2).
pub(crate) const ROCKET_CROSS_SECTIONAL_AREA_M2: f32 = 0.018241469;
/// The mass of the entire rocket at and after burnout (kg).
pub(crate) const ROCKET_BURNOUT_MASS_KG: f32 = 29.756;

/// The drag coefficient of the rocket when the flaps are closed.
pub(crate) const ROCKET_CLOSE_CD: f32 = 0.544;
/// The drag coefficient of the rocket when the flaps are fully open.
pub(crate) const ROCKET_OPEN_CD: f32 = 1.666;


/// Acceleration due to gravity at sea level (m/s^2)
pub(crate) const GRAVITY_MPS2: f32 = 9.80665;
/// Universal gas constant (J/(kmol*K))
pub(crate) const R_JPKMOLK: f32 = 8314.46;
/// The mean molar mass of air at sea level (kg/kmol)
pub(crate) const M_0_KGPKMOL: f32 = 28.9644;
/// Temperature gradient in the troposphere (K/m).
pub(crate) const L_0_KPM: f32 = -0.0065;
/// The pressure at sea level (Pa).
pub(crate) const P_0_PA: f32 = 101325.0;
/// The temperature at sea level (K).
pub(crate) const T_0_K: f32 = 288.15;

/// The density of air at sea level (kg/m^3).
pub(crate) const RHO_0_KGPM3: f32 = P_0_PA * M_0_KGPKMOL / (R_JPKMOLK * T_0_K);

/// Exponent in the barometric equation (for the troposphere).
pub(crate) const BARO_EXP: f32 = GRAVITY_MPS2 * M_0_KGPKMOL / (R_JPKMOLK * L_0_KPM);