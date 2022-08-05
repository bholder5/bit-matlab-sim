/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 *
 * axis2rot.c
 *
 * Code generation for function 'axis2rot'
 *
 */

/* Include files */
#include "axis2rot.h"
#include "main_func_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtBCInfo emlrtBCI = {
    1,                                                    /* iFirst */
    3,                                                    /* iLast */
    13,                                                   /* lineNo */
    35,                                                   /* colNo */
    "v",                                                  /* aName */
    "axis2rot",                                           /* fName */
    "/home/brad/bit-matlab-sim/Miscellaneous/axis2rot.m", /* pName */
    0                                                     /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    1,                                                    /* iFirst */
    3,                                                    /* iLast */
    19,                                                   /* lineNo */
    47,                                                   /* colNo */
    "v",                                                  /* aName */
    "axis2rot",                                           /* fName */
    "/home/brad/bit-matlab-sim/Miscellaneous/axis2rot.m", /* pName */
    0                                                     /* checkKind */
};

/* Function Definitions */
void axis2rot(const emlrtStack *sp, const real_T v[3], real_T phi,
              real_T rot[9])
{
  real_T b_sign;
  real_T cosa;
  real_T sina;
  int32_T j;
  int32_T k;
  covrtLogFcn(&emlrtCoverageInstance, 4U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 0U);
  /*  This function gives the rotation matric applied to other rotation */
  /*  matricies, not the vector (it is transpose of the rot mat applied to the
   */
  /*  vector. */
  cosa = muDoubleScalarCos(phi);
  sina = muDoubleScalarSin(phi);
  b_sign = 1.0;
  memset(&rot[0], 0, 9U * sizeof(real_T));
  for (k = 0; k < 3; k++) {
    int32_T i;
    covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 0, 1);
    i = 2 - k;
    for (j = 0; j <= i; j++) {
      real_T mij;
      int32_T b_j;
      b_j = k + j;
      covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 1, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 1U);
      if (b_j + 1 > 3) {
        emlrtDynamicBoundsCheckR2012b(b_j + 1, 1, 3, &emlrtBCI, (emlrtCTX)sp);
      }
      mij = (1.0 - cosa) * v[k] * v[b_j];
      if (covrtLogIf(&emlrtCoverageInstance, 4U, 0U, 0, k == b_j)) {
        covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 2U);
        rot[k + 3 * b_j] = mij + cosa;
      } else {
        real_T rot_tmp;
        int32_T i1;
        covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 3U);
        /* index is 3 - j - k for 0 indexed programming languages */
        i1 = 4 - (k + b_j);
        if ((i1 < 1) || (i1 > 3)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, 3, &b_emlrtBCI, (emlrtCTX)sp);
        }
        rot_tmp = b_sign * sina * v[i1 - 1];
        rot[k + 3 * b_j] = mij + rot_tmp;
        rot[b_j + 3 * k] = mij - rot_tmp;
        b_sign = -b_sign;
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b((emlrtCTX)sp);
      }
    }
    covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 1, 0);
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 0, 0);
}

/* End of code generation (axis2rot.c) */
