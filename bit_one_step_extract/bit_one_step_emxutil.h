/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: bit_one_step_emxutil.h
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 18-Jan-2022 06:50:46
 */

#ifndef BIT_ONE_STEP_EMXUTIL_H
#define BIT_ONE_STEP_EMXUTIL_H

/* Include Files */
#include "bit_one_step_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);

extern void emxFree_real_T(emxArray_real_T **pEmxArray);

extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for bit_one_step_emxutil.h
 *
 * [EOF]
 */
