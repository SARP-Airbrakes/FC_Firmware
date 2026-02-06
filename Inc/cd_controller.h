
#ifndef CD_CONTROLLER_H_
#define CD_CONTROLLER_H_

#ifdef CD_CONTROLLER_FLOAT
typedef float real_T;
#define r(x) x ## f
#define absr(x) fabsf(x)
#define powr(x,y) powf(x,y)
#define logr(x) logf(x)
#else
typedef double real_T;
#define r(x) x
#define absr(x) fabs(x)
#define powr(x,y) pow(x,y)
#define logr(x) log(x)
#endif


// Physical Constants
#define CD_CONTROLLER_AREA r(0.01824146924750467) 
#define CD_CONTROLLER_MASS r(37.651) 
#define CD_CONTROLLER_GRAVITY r(9.81) 

// Cd Limits
#define CD_CONTROLLER_MIN_CD r(0.725) 
#define CD_CONTROLLER_MAX_CD r(1.601)

// Solver Settings
#define CD_CONTROLLER_MAX_ITERS 30
#define CD_CONTROLLER_TOLERANCE r(1e-3)
#define CD_CONTROLLER_DEADZONE r(1e-6)

#ifdef __cplusplus
extern "C" {
#endif

real_T cd_controller_solve(real_T vel_mps, real_T alt_m, real_T target_m);

#ifdef __cplusplus
};
#endif

#endif
