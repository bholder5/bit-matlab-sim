/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * axis2rot.c
 *
 * Code generation for function 'axis2rot'
 *
 */

/* Include files */
#include "axis2rot.h"
#include "bit_one_step_mex_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtBCInfo emlrtBCI = {
    1,                                                       /* iFirst */
    3,                                                       /* iLast */
    13,                                                      /* lineNo */
    35,                                                      /* colNo */
    "v",                                                     /* aName */
    "axis2rot",                                              /* fName */
    "/home/bholder/bit-matlab-sim/Miscellaneous/axis2rot.m", /* pName */
    0                                                        /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = {
    1,                                                       /* iFirst */
    3,                                                       /* iLast */
    19,                                                      /* lineNo */
    47,                                                      /* colNo */
    "v",                                                     /* aName */
    "axis2rot",                                              /* fName */
    "/home/bholder/bit-matlab-sim/Miscellaneous/axis2rot.m", /* pName */
    0                                                        /* checkKind */
};

/* Function Definitions */
/*
 * function rot = axis2rot( v, phi)
 */
void axis2rot(const emlrtStack *sp, const real_T v[3], real32_T phi,
              real_T rot[9])
{
  real_T b_sign;
  int32_T j;
  int32_T k;
  real32_T cosa;
  real32_T sina;
  covrtLogFcn(&emlrtCoverageInstance, 9U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 0U);
  /*  This function gives the rotation matric applied to other rotation */
  /*  matricies, not the vector (it is transpose of the rot mat applied to the
   */
  /*  vector. */
  /* 'axis2rot:5' cosa = cos(phi); */
  cosa = muSingleScalarCos(phi);
  /* 'axis2rot:6' sina = sin(phi); */
  sina = muSingleScalarSin(phi);
  /* 'axis2rot:8' sign = 1; */
  b_sign = 1.0;
  /* 'axis2rot:9' rot = (zeros(3,3)); */
  memset(&rot[0], 0, 9U * sizeof(real_T));
  /* 'axis2rot:11' for k = 1:3 */
  for (k = 0; k < 3; k++) {
    int32_T i;
    covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 0, 1);
    /* 'axis2rot:12' for j = k:3 */
    i = 2 - k;
    for (j = 0; j <= i; j++) {
      int32_T b_j;
      real32_T mij;
      b_j = k + j;
      covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 1, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 1U);
      /* 'axis2rot:13' mij = (1-cosa)*v(k)*v(j); */
      if (b_j + 1 > 3) {
        emlrtDynamicBoundsCheckR2012b(b_j + 1, 1, 3, &emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      mij = (1.0F - cosa) * (real32_T)v[k] * (real32_T)v[b_j];
      /* 'axis2rot:14' if (k == j) */
      if (covrtLogIf(&emlrtCoverageInstance, 9U, 0U, 0, k == b_j)) {
        covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 2U);
        /* 'axis2rot:15' mij = mij + cosa; */
        mij += cosa;
        /* 'axis2rot:16' rot(k,j) = mij; */
        rot[k + 3 * b_j] = mij;
      } else {
        int32_T i1;
        real32_T rot_tmp;
        covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 3U);
        /* 'axis2rot:17' else */
        /* index is 3 - j - k for 0 indexed programming languages */
        /* 'axis2rot:19' rot(k,j) = mij + (sign*sina*v((5-k-j)+1)); */
        i1 = 4 - (k + b_j);
        if ((i1 < 1) || (i1 > 3)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, 3, &b_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        rot_tmp = (real32_T)b_sign * sina * (real32_T)v[i1 - 1];
        rot[k + 3 * b_j] = mij + rot_tmp;
        /* 'axis2rot:20' rot(j,k) = mij - (sign*sina*v((5-k-j)+1)); */
        rot[b_j + 3 * k] = mij - rot_tmp;
        /* 'axis2rot:21' sign = sign*-1; */
        b_sign = -b_sign;
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b((emlrtConstCTX)sp);
      }
    }
    covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 1, 0);
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 0, 0);
}

/*
 * function rot = axis2rot( v, phi)
 */
void b_axis2rot(const emlrtStack *sp, const real_T v[3], real_T phi,
                real_T rot[9])
{
  real_T b_sign;
  real_T cosa;
  real_T sina;
  int32_T j;
  int32_T k;
  covrtLogFcn(&emlrtCoverageInstance, 9U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 0U);
  /*  This function gives the rotation matric applied to other rotation */
  /*  matricies, not the vector (it is transpose of the rot mat applied to the
   */
  /*  vector. */
  /* 'axis2rot:5' cosa = cos(phi); */
  cosa = muDoubleScalarCos(phi);
  /* 'axis2rot:6' sina = sin(phi); */
  sina = muDoubleScalarSin(phi);
  /* 'axis2rot:8' sign = 1; */
  b_sign = 1.0;
  /* 'axis2rot:9' rot = (zeros(3,3)); */
  memset(&rot[0], 0, 9U * sizeof(real_T));
  /* 'axis2rot:11' for k = 1:3 */
  for (k = 0; k < 3; k++) {
    int32_T i;
    covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 0, 1);
    /* 'axis2rot:12' for j = k:3 */
    i = 2 - k;
    for (j = 0; j <= i; j++) {
      real_T mij;
      int32_T b_j;
      b_j = k + j;
      covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 1, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 1U);
      /* 'axis2rot:13' mij = (1-cosa)*v(k)*v(j); */
      if (b_j + 1 > 3) {
        emlrtDynamicBoundsCheckR2012b(b_j + 1, 1, 3, &emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
      mij = (1.0 - cosa) * v[k] * v[b_j];
      /* 'axis2rot:14' if (k == j) */
      if (covrtLogIf(&emlrtCoverageInstance, 9U, 0U, 0, k == b_j)) {
        covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 2U);
        /* 'axis2rot:15' mij = mij + cosa; */
        rot[k + 3 * b_j] = mij + cosa;
        /* 'axis2rot:16' rot(k,j) = mij; */
      } else {
        real_T rot_tmp;
        int32_T i1;
        covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 3U);
        /* 'axis2rot:17' else */
        /* index is 3 - j - k for 0 indexed programming languages */
        /* 'axis2rot:19' rot(k,j) = mij + (sign*sina*v((5-k-j)+1)); */
        i1 = 4 - (k + b_j);
        if ((i1 < 1) || (i1 > 3)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, 3, &b_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        rot_tmp = b_sign * sina * v[i1 - 1];
        rot[k + 3 * b_j] = mij + rot_tmp;
        /* 'axis2rot:20' rot(j,k) = mij - (sign*sina*v((5-k-j)+1)); */
        rot[b_j + 3 * k] = mij - rot_tmp;
        /* 'axis2rot:21' sign = sign*-1; */
        b_sign = -b_sign;
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b((emlrtConstCTX)sp);
      }
    }
    covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 1, 0);
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 0, 0);
}

/* End of code generation (axis2rot.c) */
