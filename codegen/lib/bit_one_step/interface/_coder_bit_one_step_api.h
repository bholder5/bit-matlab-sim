/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_bit_one_step_api.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 22-Sep-2024 08:32:20
 */

#ifndef _CODER_BIT_ONE_STEP_API_H
#define _CODER_BIT_ONE_STEP_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void bit_one_step(real_T x0[21], real_T tau_applied[9], real_T unlock[9],
                  real_T w_piv, uint8_T piv_flag, real_T dt, uint16_T num_steps,
                  real_T tau_max_piv, real_T thet_pit_nom, real_T x_flex0[104],
                  real_T tau_flex[5], uint8_T flexure_flag, uint8_T sb_flag,
                  real_T y_true[21], real_T y_flex[104]);

void bit_one_step_api(const mxArray *const prhs[13], int32_T nlhs,
                      const mxArray *plhs[2]);

void bit_one_step_atexit(void);

void bit_one_step_initialize(void);

void bit_one_step_terminate(void);

void bit_one_step_xil_shutdown(void);

void bit_one_step_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_bit_one_step_api.h
 *
 * [EOF]
 */
