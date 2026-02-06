
#include "cd_controller.h"

#include <math.h>

real_T cd_controller_solve(real_T vel_mps, real_T alt_m, real_T target_m)
{
    // modelling atmosphere characteristics at current altitude
    // uses the ISA static atmosphere model
    real_T temp = r(15.04) - r(0.00649) * alt_m + r(273.1); // deg K
    real_T press = r(101.29) * powr(temp / r(288.08), r(5.256)); // kPa
    real_T density = press / (r(0.2869) * temp);

    int i;

    if (vel_mps < CD_CONTROLLER_MAX_VEL && alt_m > CD_CONTROLLER_MIN_ALT) {
        real_T cdab = CD_CONTROLLER_RANGE > r(0.2) ? r(0.2) : CD_CONTROLLER_RANGE;

        for (i = 0; i < CD_CONTROLLER_MAX_ITERS; i++) {
            real_T cd_sim = CD_CONTROLLER_MIN_CD + cdab;
            real_T k = r(0.5) * density * cd_sim * CD_CONTROLLER_AREA;

            // for numerical stability
            if (k < CD_CONTROLLER_K_TOLERANCE)
                k = CD_CONTROLLER_K_TOLERANCE;
            
            real_T predicted_alt = alt_m + (CD_CONTROLLER_MASS / (2 * k)) * 
                logr((k * vel_mps * vel_mps) / 
                        (CD_CONTROLLER_MASS * CD_CONTROLLER_GRAVITY) + 1);
            real_T residual = predicted_alt - target_m;

            // derivative of k over Cd
            real_T dk_dcd = r(0.5) * density * CD_CONTROLLER_AREA;

            real_T term_2 = (k * vel_mps * vel_mps) / (CD_CONTROLLER_MASS * CD_CONTROLLER_GRAVITY) + 1;
            real_T term_1 = logr(term_2);

            // derivative of alt over k
            real_T dalt_dk = -CD_CONTROLLER_MASS * term_1 / (2 * k * k) +
                (CD_CONTROLLER_MASS / (2 * k)) * (vel_mps * vel_mps /
                        (CD_CONTROLLER_MASS * CD_CONTROLLER_GRAVITY)) / term_2;
            real_T dresidual_dcd = dalt_dk * dk_dcd;

            if (absr(dresidual_dcd) < CD_CONTROLLER_DEADZONE) {
                break;
            }
            
            real_T new_cdab = cdab - residual / dresidual_dcd;

            if (new_cdab > CD_CONTROLLER_RANGE)
                new_cdab = CD_CONTROLLER_RANGE;
            else if (new_cdab < 0)
                new_cdab = 0;
            
            if (absr(new_cdab - cdab) < CD_CONTROLLER_TOLERANCE) {
                cdab = new_cdab;
                break;
            }
            cdab = new_cdab;
        }
        return CD_CONTROLLER_MIN_CD + cdab;
    } 
}
