/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 *
 * main_func_data.c
 *
 * Code generation for function 'main_func_data'
 *
 */

/* Include files */
#include "main_func_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131626U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "main_func",                                          /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

const real_T dv[27] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -30.5,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.4,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

const int16_T iv[9] = {0, 0, 10, 0, 0, 1, 350, 73, 150};

const int8_T iv1[27] = {0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1,
                        0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0};

covrtInstance emlrtCoverageInstance;

/* End of code generation (main_func_data.c) */
