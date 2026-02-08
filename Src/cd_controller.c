
#include "cd_controller.h"
#include <math.h>

real_T cd_controller_solve(real_T vel_mps, real_T alt_m, real_T target_m, real_T currentTime)
{
    // ------------------ ISA Atmosphere ------------------
    real_T temp = r(15.04) - r(0.00649) * alt_m + r(273.1);
    real_T press = r(101.29) * powr(temp / r(288.08), r(5.256));
    real_T density = press / (r(0.2869) * temp);

    // Initial guess
    real_T cd_curr = CD_CONTROLLER_MIN_CD + r(0.2);

    if (cd_curr > CD_CONTROLLER_MAX_CD) cd_curr = CD_CONTROLLER_MAX_CD;

    for (int i = 0; i < CD_CONTROLLER_MAX_ITERS; i++) {

        real_T k = r(0.5) * density * cd_curr * CD_CONTROLLER_AREA;
        if (k < r(1e-9)) k = r(1e-9);

        real_T v2 = vel_mps * vel_mps;
        real_T mg = CD_CONTROLLER_MASS * CD_CONTROLLER_GRAVITY;

        real_T term2 = (k * v2 / mg) + r(1.0);
        real_T term1 = logr(term2);

        // Apogee prediction
        real_T predicted_alt = alt_m + (CD_CONTROLLER_MASS / (r(2.0) * k)) * term1;

        // Residual
        real_T residual = predicted_alt - target_m;

        // Derivative d(apogee)/dk
        real_T dalt_dk =
            -CD_CONTROLLER_MASS * term1 / (r(2.0) * k * k) +
             (CD_CONTROLLER_MASS / (r(2.0) * k)) * (v2 / mg) / term2;

        // Chain rule: d(apogee)/dCd
        real_T dk_dcd = r(0.5) * density * CD_CONTROLLER_AREA;
        real_T df_dcd = dalt_dk * dk_dcd;

        if (absr(df_dcd) < CD_CONTROLLER_DEADZONE)
            break;

        // Newton update
        real_T cd_new = cd_curr - (residual / df_dcd);

        // Bound Cd
        if (cd_new > CD_CONTROLLER_MAX_CD) cd_new = CD_CONTROLLER_MAX_CD;
        if (cd_new < CD_CONTROLLER_MIN_CD) cd_new = CD_CONTROLLER_MIN_CD;

        if (absr(cd_new - cd_curr) < CD_CONTROLLER_TOLERANCE) {
            cd_curr = cd_new;
            break;
        }

        cd_curr = cd_new;
    }

    return cd_curr;
}