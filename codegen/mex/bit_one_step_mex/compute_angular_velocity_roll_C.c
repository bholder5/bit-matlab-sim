/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_angular_velocity_roll_C.c
 *
 * Code generation for function 'compute_angular_velocity_roll_C'
 *
 */

/* Include files */
#include "compute_angular_velocity_roll_C.h"
#include "axis2rot.h"
#include "bit_one_step_mex_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo sc_emlrtRSI = {
    9,                                 /* lineNo */
    "compute_angular_velocity_roll_C", /* fcnName */
    "/home/bholder/bit-matlab-sim/compute_angular_velocity_roll_C.m" /* pathName
                                                                      */
};

/* Function Definitions */
/*
 * function [omega] = compute_angular_velocity_roll_C(x, z_n)
 */
void compute_angular_velocity_roll_C(const emlrtStack *sp, const real_T x[18],
                                     const real_T z_n[27], real_T omega[3])
{
  emlrtStack st;
  real_T b_Cn[24];
  real_T s8[24];
  real_T d;
  int32_T b_i;
  int32_T i;
  int32_T s8_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  covrtLogFcn(&emlrtCoverageInstance, 14U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 14U, 0U);
  /* UNTITLED2 Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* 'compute_angular_velocity_roll_C:4' theta = x(10:18); */
  /* 'compute_angular_velocity_roll_C:5' dtheta = x(1:8); */
  /* 'compute_angular_velocity_roll_C:7' s8 = zeros(3,8); */
  memset(&s8[0], 0, 24U * sizeof(real_T));
  /* 'compute_angular_velocity_roll_C:8' for i = 1:8 */
  for (i = 0; i < 8; i++) {
    real_T Cn[9];
    covrtLogFor(&emlrtCoverageInstance, 14U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 14U, 1U);
    /* 'compute_angular_velocity_roll_C:9' Cn = axis2rot(z_n(:,i), theta(i)); */
    st.site = &sc_emlrtRSI;
    axis2rot(&st, &z_n[3 * i], x[i + 9], Cn);
    /* 'compute_angular_velocity_roll_C:10' s8(:,i) = z_n(:,i); */
    s8[3 * i] = z_n[3 * i];
    s8_tmp = 3 * i + 1;
    s8[s8_tmp] = z_n[s8_tmp];
    s8_tmp = 3 * i + 2;
    s8[s8_tmp] = z_n[s8_tmp];
    /* 'compute_angular_velocity_roll_C:11' s8 = Cn*s8; */
    for (s8_tmp = 0; s8_tmp < 3; s8_tmp++) {
      real_T d1;
      real_T d2;
      d = Cn[s8_tmp];
      d1 = Cn[s8_tmp + 3];
      d2 = Cn[s8_tmp + 6];
      for (b_i = 0; b_i < 8; b_i++) {
        b_Cn[s8_tmp + 3 * b_i] =
            (d * s8[3 * b_i] + d1 * s8[3 * b_i + 1]) + d2 * s8[3 * b_i + 2];
      }
    }
    memcpy(&s8[0], &b_Cn[0], 24U * sizeof(real_T));
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 14U, 0U, 0, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 14U, 2U);
  /* 'compute_angular_velocity_roll_C:14' omega = s8 * dtheta; */
  for (s8_tmp = 0; s8_tmp < 3; s8_tmp++) {
    d = 0.0;
    for (b_i = 0; b_i < 8; b_i++) {
      d += s8[s8_tmp + 3 * b_i] * x[b_i];
    }
    omega[s8_tmp] = d;
  }
}

/* End of code generation (compute_angular_velocity_roll_C.c) */
