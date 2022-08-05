/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: bit_one_step_types.h
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 18-Jan-2022 06:50:46
 */

#ifndef BIT_ONE_STEP_TYPES_H
#define BIT_ONE_STEP_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#endif
/*
 * File trailer for bit_one_step_types.h
 *
 * [EOF]
 */
