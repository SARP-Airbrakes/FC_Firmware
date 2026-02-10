#define S_FUNCTION_NAME  simulink_sfunction
#define S_FUNCTION_LEVEL 2
#define MDL_START

#include "simstruc.h"
#include "cd_controller.h"

static void mdlStart(SimStruct *S) {}

static void mdlInitializeSizes(SimStruct *S)
{
    if (!ssSetNumInputPorts(S, 4)) return;
    if (!ssSetNumOutputPorts(S, 1)) return;

    for (int i = 0; i < 4; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 1);
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    ssSetOutputPortWidth(S, 0, 1);

    ssSetNumSampleTimes(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    
    const real_T* V        = ssGetInputPortRealSignal(S, 0);
    const real_T* X        = ssGetInputPortRealSignal(S, 1);
    const real_T* tgt_alt  = ssGetInputPortRealSignal(S, 2);
    const real_T* time     = ssGetInputPortRealSignal(S, 3);

    // for debugging
    ssPrintf("V=%.2f X=%.2f tgt=%.2f t=%.2f\n", *V, *X, *tgt_alt, *time);

    real_T* Cd = ssGetOutputPortRealSignal(S, 0);

    // ------------------ MATLAB Gate Logic ------------------
    if (*V < r(210.0) && *time > r(10.0) && *X > r(6000.0)) {
        *Cd = cd_controller_solve(*V, *X, *tgt_alt, *time);
    } else {
        *Cd = CD_CONTROLLER_MIN_CD;
    }

}

static void mdlTerminate(SimStruct *S) {}

#include "simulink.c"