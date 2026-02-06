
#ifndef CD_CONTROLLER_H_
#define CD_CONTROLLER_H_

#ifdef CD_CONTROLLER_FLOAT
typedef float real_T;
#define r(x) x ## f
#define powr(x,y) powf(x,y)
#define logr(x) logf(x)
#else
typedef double real_T;
#define r(x) x
#define powr(x,y) pow(x,y)
#define logr(x) log(x)
#endif

#define absr(x) (x < 0 ? -x : x)

#define CD_CONTROLLER_AREA r(0.01824146924750467) // airbrakes area (m^2)
#define CD_CONTROLLER_MASS r(37.651) // burnout mass (kg)
#define CD_CONTROLLER_GRAVITY r(9.81) // acceleration due to gravity (m/s^2)

#define CD_CONTROLLER_MAX_CD r(1.601) // maximum possible CD (extended)
#define CD_CONTROLLER_MIN_CD r(0.725) // minimum possible CD (retracted)
#define CD_CONTROLLER_RANGE (CD_CONTROLLER_MAX_CD - CD_CONTROLLER_MIN_CD)

#define CD_CONTROLLER_MAX_VEL r(210) // maximum velocity (m/s)
#define CD_CONTROLLER_MIN_ALT r(6000) // minimum altitude for activation (m)
#define CD_CONTROLLER_MAX_ITERS 30
#define CD_CONTROLLER_TOLERANCE r(1e-3)
#define CD_CONTROLLER_K_TOLERANCE r(1e-9)
#define CD_CONTROLLER_DEADZONE r(1e-6)

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * Finds the best Cd value to reach a target altitude given current inertial
 * parameters, assuming that the airbrakes are ready to be deployed.
 *
 * vel_mps is the current velocity in m/s
 * alt_m is the current altitude in m
 * target_m is the target altitude in m
 */
real_T cd_controller_solve(real_T vel_mps, real_T alt_m, real_T target_m);

#ifdef __cplusplus
};
#endif // __cplusplus

#endif // CD_CONTROLLER_H_
