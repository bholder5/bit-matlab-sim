/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_rotation_mat_C.c
 *
 * Code generation for function 'compute_rotation_mat_C'
 *
 */

/* Include files */
#include "compute_rotation_mat_C.h"
#include "axis2rot.h"
#include "bit_one_step_mex_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo uc_emlrtRSI = {
    6,                                                      /* lineNo */
    "compute_rotation_mat_C",                               /* fcnName */
    "/home/bholder/bit-matlab-sim/compute_rotation_mat_C.m" /* pathName */
};

/* Function Definitions */
/*
 * function [C] = compute_rotation_mat_C(z_n, theta)
 */
void compute_rotation_mat_C(const emlrtStack *sp, const real_T z_n[27],
                            const real_T theta[9], real_T C[9])
{
  emlrtStack st;
  real_T a[9];
  real_T b_a[9];
  int32_T b_i;
  int32_T i;
  int32_T i1;
  st.prev = sp;
  st.tls = sp->tls;
  covrtLogFcn(&emlrtCoverageInstance, 16U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 16U, 0U);
  /* UNTITLED3 Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* 'compute_rotation_mat_C:4' C = (eye(3)); */
  memset(&C[0], 0, 9U * sizeof(real_T));
  C[0] = 1.0;
  C[4] = 1.0;
  C[8] = 1.0;
  /* 'compute_rotation_mat_C:5' for i = 1:9 */
  for (i = 0; i < 9; i++) {
    covrtLogFor(&emlrtCoverageInstance, 16U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 16U, 1U);
    /* 'compute_rotation_mat_C:6' C = axis2rot(z_n(:,i), theta(i)) * C; */
    st.site = &uc_emlrtRSI;
    axis2rot(&st, &z_n[3 * i], theta[i], b_a);
    for (b_i = 0; b_i < 3; b_i++) {
      real_T d;
      real_T d1;
      real_T d2;
      d = b_a[b_i];
      d1 = b_a[b_i + 3];
      d2 = b_a[b_i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        a[b_i + 3 * i1] =
            (d * C[3 * i1] + d1 * C[3 * i1 + 1]) + d2 * C[3 * i1 + 2];
      }
    }
    memcpy(&C[0], &a[0], 9U * sizeof(real_T));
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 16U, 0U, 0, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 16U, 2U);
  /* 'compute_rotation_mat_C:8' C = C'; */
  for (b_i = 0; b_i < 3; b_i++) {
    b_a[3 * b_i] = C[b_i];
    b_a[3 * b_i + 1] = C[b_i + 3];
    b_a[3 * b_i + 2] = C[b_i + 6];
  }
  memcpy(&C[0], &b_a[0], 9U * sizeof(real_T));
}

/* End of code generation (compute_rotation_mat_C.c) */
