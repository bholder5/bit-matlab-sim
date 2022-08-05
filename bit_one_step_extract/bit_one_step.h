/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: bit_one_step.h
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 18-Jan-2022 06:50:46
 */

#ifndef BIT_ONE_STEP_H
#define BIT_ONE_STEP_H

/* Include Files */
#include "bit_one_step_types.h"
#include "rtwtypes.h"
#include "omp.h"
#include <stddef.h>
#include <stdlib.h>

/* Variable Declarations */
extern omp_nest_lock_t bit_one_step_nestLockGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void bit_one_step(const double x0[21], const double tau_applied[9],
                         double dt, double tf, emxArray_real_T *y_all);

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
