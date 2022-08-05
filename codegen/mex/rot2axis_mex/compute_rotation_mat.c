/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_rotation_mat.c
 *
 * Code generation for function 'compute_rotation_mat'
 *
 */

/* Include files */
#include "compute_rotation_mat.h"
#include "axis2rot.h"
#include "rot2axis_mex_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo tb_emlrtRSI = {
    6,                      /* lineNo */
    "compute_rotation_mat", /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/compute_rotation_mat.m" /* pathName
                                                                        */
};

/* Function Definitions */
void compute_rotation_mat(const emlrtStack *sp, const real_T z_n[27],
                          const real_T theta[9], real_T C[9])
{
  emlrtStack st;
  real_T a[9];
  real_T b_a[9];
  real_T d;
  real_T d1;
  real_T d2;
  int32_T b_i;
  int32_T i;
  int32_T i1;
  st.prev = sp;
  st.tls = sp->tls;
  covrtLogFcn(&emlrtCoverageInstance, 8U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 8U, 0U);
  /* UNTITLED3 Summary of this function goes here */
  /*    Detailed explanation goes here */
  memset(&C[0], 0, 9U * sizeof(real_T));
  C[0] = 1.0;
  C[4] = 1.0;
  C[8] = 1.0;
  for (i = 0; i < 9; i++) {
    covrtLogFor(&emlrtCoverageInstance, 8U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 8U, 1U);
    st.site = &tb_emlrtRSI;
    axis2rot(&st, *(real_T(*)[3]) & z_n[3 * i], theta[i], a);
    for (b_i = 0; b_i < 3; b_i++) {
      d = a[b_i];
      d1 = a[b_i + 3];
      d2 = a[b_i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        b_a[b_i + 3 * i1] =
            (d * C[3 * i1] + d1 * C[3 * i1 + 1]) + d2 * C[3 * i1 + 2];
      }
    }
    memcpy(&C[0], &b_a[0], 9U * sizeof(real_T));
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 8U, 0U, 0, 0);
}

/* End of code generation (compute_rotation_mat.c) */
