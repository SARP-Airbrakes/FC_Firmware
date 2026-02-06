
#define S_FUNCTION_NAME  controller_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "cd_controller.h"

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumInputPorts(S, 3);
    ssSetNumOutputPorts(S, 1);

    for (int i = 0; i < 4; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 1);
    }

    ssSetOutputPortWidth(S, 0, 1);

    ssSetNumSampleTimes(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
}

static void mdlStart(SimStruct *S)
{
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    const real_T* V        = ssGetInputPortRealSignal(S, 0);
    const real_T* X        = ssGetInputPortRealSignal(S, 1);
    const real_T* tgt_alt  = ssGetInputPortRealSignal(S, 2);

    real_T* Cd = ssGetOutputPortRealSignal(S, 0);

    real_T out = cd_controller_solve(*V, *X, *tgt_alt);
    *Cd = out;
}

static void mdlTerminate(SimStruct *S)
{
}

#include "simulink.c"
