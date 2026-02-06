
#define S_FUNCTION_NAME  simulink_sfunction
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "cd_controller.h"

static void mdlInitializeSizes(SimStruct *S)
{
    // 1. Ensure we have exactly 4 input ports
    if (!ssSetNumInputPorts(S, 4)) return;
    if (!ssSetNumOutputPorts(S, 1)) return;

    for (int i = 0; i < 4; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 1);
        
        // 2. FORCE DOUBLE PRECISION: This prevents the "All Zeros" issue 
        // if your Simulink signals are 'single' or 'int'.
        ssSetInputPortDataType(S, i, SS_DOUBLE);
        
        // 3. FORCE CONTIGUOUS: Required for ssGetInputPortRealSignal
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE); // Force output to double
    ssSetNumSampleTimes(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Use InputRealPtrsType for safer access in some Simulink configurations
    InputRealPtrsType uPtrsV   = ssGetInputPortRealSignalPtrs(S, 0);
    InputRealPtrsType uPtrsX   = ssGetInputPortRealSignalPtrs(S, 1);
    InputRealPtrsType uPtrsTgt = ssGetInputPortRealSignalPtrs(S, 2);
    InputRealPtrsType uPtrsTim = ssGetInputPortRealSignalPtrs(S, 3);

    // Dereference the pointers to get the actual values
    real_T V       = *uPtrsV[0];
    real_T X       = *uPtrsX[0];
    real_T tgt_alt = *uPtrsTgt[0];
    real_T time    = *uPtrsTim[0];

    real_T* Cd = ssGetOutputPortRealSignal(S, 0);

    // Debugging: This will print the actual values to the MATLAB Command Window
    // If these are still 0, the issue is the wire connection in Simulink.
    // ssPrintf("V: %f, X: %f, Tgt: %f, Time: %f\n", V, X, tgt_alt, time);

    if (V < 210.0 && time > 10.0 && V > 0.0 && X > 6000.0) {
        *Cd = cd_controller_solve(V, X, tgt_alt);
    } else {
        *Cd = CD_CONTROLLER_MIN_CD;
    }
}

static void mdlTerminate(SimStruct *S) {}

#include "simulink.c"
