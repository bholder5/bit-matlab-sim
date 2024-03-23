/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_angular_velocity_C.c
 *
 * Code generation for function 'compute_angular_velocity_C'
 *
 */

/* Include files */
#include "compute_angular_velocity_C.h"
#include "axis2rot.h"
#include "bit_one_step_mex_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo nc_emlrtRSI = {
    9,                                                          /* lineNo */
    "compute_angular_velocity_C",                               /* fcnName */
    "/home/bholder/bit-matlab-sim/compute_angular_velocity_C.m" /* pathName */
};

/* Function Definitions */
/*
 * function [omega] = compute_angular_velocity_C(x, z_n)
 */
void compute_angular_velocity_C(const emlrtStack *sp, const real_T x[18],
                                const real_T z_n[27], real_T omega[3])
{
  emlrtStack st;
  real_T b_Cn[27];
  real_T s9[27];
  real_T d;
  int32_T b_i;
  int32_T i;
  int32_T s9_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  covrtLogFcn(&emlrtCoverageInstance, 13U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 13U, 0U);
  /* UNTITLED2 Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* 'compute_angular_velocity_C:4' theta = x(10:18); */
  /* 'compute_angular_velocity_C:5' dtheta = x(1:9); */
  /* 'compute_angular_velocity_C:7' s9 = zeros(3,9); */
  memset(&s9[0], 0, 27U * sizeof(real_T));
  /* 'compute_angular_velocity_C:8' for i = 1:9 */
  for (i = 0; i < 9; i++) {
    real_T Cn[9];
    covrtLogFor(&emlrtCoverageInstance, 13U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 13U, 1U);
    /* 'compute_angular_velocity_C:9' Cn = axis2rot(z_n(:,i), theta(i)); */
    st.site = &nc_emlrtRSI;
    b_axis2rot(&st, &z_n[3 * i], x[i + 9], Cn);
    /* 'compute_angular_velocity_C:10' s9(:,i) = z_n(:,i); */
    s9[3 * i] = z_n[3 * i];
    s9_tmp = 3 * i + 1;
    s9[s9_tmp] = z_n[s9_tmp];
    s9_tmp = 3 * i + 2;
    s9[s9_tmp] = z_n[s9_tmp];
    /* 'compute_angular_velocity_C:11' s9 = Cn*s9; */
    for (s9_tmp = 0; s9_tmp < 3; s9_tmp++) {
      real_T d1;
      real_T d2;
      d = Cn[s9_tmp];
      d1 = Cn[s9_tmp + 3];
      d2 = Cn[s9_tmp + 6];
      for (b_i = 0; b_i < 9; b_i++) {
        b_Cn[s9_tmp + 3 * b_i] =
            (d * s9[3 * b_i] + d1 * s9[3 * b_i + 1]) + d2 * s9[3 * b_i + 2];
      }
    }
    memcpy(&s9[0], &b_Cn[0], 27U * sizeof(real_T));
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 13U, 0U, 0, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 13U, 2U);
  /* 'compute_angular_velocity_C:14' omega = s9 * dtheta; */
  for (s9_tmp = 0; s9_tmp < 3; s9_tmp++) {
    d = 0.0;
    for (b_i = 0; b_i < 9; b_i++) {
      d += s9[s9_tmp + 3 * b_i] * x[b_i];
    }
    omega[s9_tmp] = d;
  }
}

/* End of code generation (compute_angular_velocity_C.c) */
