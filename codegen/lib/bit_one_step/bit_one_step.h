/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: bit_one_step.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 22-Sep-2024 08:32:20
 */

#ifndef BIT_ONE_STEP_H
#define BIT_ONE_STEP_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void bit_one_step(const double x0[21], double tau_applied[9],
                         const double unlock[9], double w_piv,
                         unsigned char piv_flag, double dt,
                         unsigned short num_steps, double tau_max_piv,
                         double thet_pit_nom, const double x_flex0[104],
                         const double tau_flex[5], unsigned char flexure_flag,
                         unsigned char sb_flag, double y_true[21],
                         double y_flex[104]);

extern void bit_one_step_initialize(void);

extern void bit_one_step_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for bit_one_step.h
 *
 * [EOF]
 */
