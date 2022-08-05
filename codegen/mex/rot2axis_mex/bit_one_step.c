/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * bit_one_step.c
 *
 * Code generation for function 'bit_one_step'
 *
 */

/* Include files */
#include "bit_one_step.h"
#include "axis2rot.h"
#include "bit_propagator.h"
#include "compute_rotation_mat.h"
#include "rot2axis.h"
#include "rot2axis_mex_data.h"
#include "rt_nonfinite.h"
#include "xzsvdc.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    30,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    34,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    38,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    42,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    61,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo f_emlrtRSI = {
    62,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    64,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI = {
    65,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo i_emlrtRSI = {
    66,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    71,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo vb_emlrtRSI = {
    49,     /* lineNo */
    "norm", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/lib/matlab/matfun/norm.m" /* pathName
                                                                     */
};

static emlrtRSInfo wb_emlrtRSI = {
    71,         /* lineNo */
    "mat2norm", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/lib/matlab/matfun/norm.m" /* pathName
                                                                     */
};

static emlrtRSInfo xb_emlrtRSI = {
    20,    /* lineNo */
    "svd", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/svd.m" /* pathName
                                                                       */
};

static emlrtRSInfo yb_emlrtRSI = {
    99,           /* lineNo */
    "callLAPACK", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/svd.m" /* pathName
                                                                       */
};

static emlrtRSInfo ac_emlrtRSI =
    {
        34,       /* lineNo */
        "xgesvd", /* fcnName */
        "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgesvd.m" /* pathName */
};

/* Function Definitions */
void bit_one_step(const emlrtStack *sp, const real_T x0[21],
                  const real_T tau_applied[9], real_T y_true[21],
                  real_T phi_true[3], real_T w_k_true[3])
{
  static const real_T dv[27] = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                                0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  real_T y_all[210];
  real_T a[21];
  real_T b_y_true[21];
  real_T k1[21];
  real_T k2[21];
  real_T k3[21];
  real_T Psik[9];
  real_T ck0[9];
  real_T ck1[9];
  real_T s[3];
  real_T absx;
  real_T y;
  real_T y_true_tmp;
  int32_T i;
  int32_T step;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  covrtLogFcn(&emlrtCoverageInstance, 0U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 0U);
  /*  Run initialization script */
  /*     %% Setup Simulation */
  /*  initial conditions, state is dtheta; theta */
  memcpy(&y_true[0], &x0[0], 21U * sizeof(real_T));
  /*  Sim Parameters */
  /*  y_all1 = zeros(18, tf/(dt)); */
  /*  sim */
  for (step = 0; step < 10; step++) {
    covrtLogFor(&emlrtCoverageInstance, 0U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 1U);
    /*          fprintf('time is: %f \n', t_vec(step)) */
    /*          if ~mod(t_vec(step), 1000) */
    /*              timeis = t_vec(step) */
    /*          end */
    /*         %% Propagate the system */
    /* RK4 solver */
    memcpy(&ck0[0], &tau_applied[0], 9U * sizeof(real_T));
    st.site = &emlrtRSI;
    bit_propagator(&st, y_true, ck0, k1);
    for (i = 0; i < 21; i++) {
      y_true_tmp = k1[i] * 0.0001;
      k1[i] = y_true_tmp;
      b_y_true[i] = y_true[i] + y_true_tmp / 2.0;
    }
    memcpy(&ck0[0], &tau_applied[0], 9U * sizeof(real_T));
    st.site = &b_emlrtRSI;
    bit_propagator(&st, b_y_true, ck0, k2);
    for (i = 0; i < 21; i++) {
      y_true_tmp = k2[i] * 0.0001;
      k2[i] = y_true_tmp;
      b_y_true[i] = y_true[i] + y_true_tmp / 2.0;
    }
    memcpy(&ck0[0], &tau_applied[0], 9U * sizeof(real_T));
    st.site = &c_emlrtRSI;
    bit_propagator(&st, b_y_true, ck0, k3);
    for (i = 0; i < 21; i++) {
      y_true_tmp = k3[i] * 0.0001;
      k3[i] = y_true_tmp;
      b_y_true[i] = y_true[i] + y_true_tmp;
    }
    memcpy(&ck0[0], &tau_applied[0], 9U * sizeof(real_T));
    st.site = &d_emlrtRSI;
    bit_propagator(&st, b_y_true, ck0, a);
    for (i = 0; i < 21; i++) {
      y_true[i] +=
          (((k1[i] + 2.0 * k2[i]) + 2.0 * k3[i]) + a[i] * 0.0001) / 6.0;
    }
    for (i = 0; i < 9; i++) {
      y_true_tmp = y_true[i + 9];
      y_true[i + 9] =
          (y_true_tmp +
           6.2831853071795862 * (real_T)(y_true_tmp < -3.1415926535897931)) -
          6.2831853071795862 * (real_T)(y_true_tmp > 3.1415926535897931);
    }
    memcpy(&y_all[step * 21], &y_true[0], 21U * sizeof(real_T));
    /*          fprintf('current state:  %0.10f  %0.10f %0.10f %0.10f %0.10f
     * %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f
     * %0.10f %0.10f %0.10f  %0.10f %0.10f %0.10f \n', ... */
    /*              y_true(1), y_true(2), y_true(3), y_true(4), y_true(5),
     * y_true(6),... */
    /*               y_true(7), y_true(8), y_true(9), y_true(10), y_true(11),
     * y_true(12),... */
    /*                y_true(13), y_true(14), y_true(15), y_true(16),
     * y_true(17), y_true(18),... */
    /*                 y_true(19), y_true(20), y_true(21));         */
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 0U, 0U, 0, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 2U);
  /*     %% calculate current yaw pitch roll, and current actual gyros */
  st.site = &e_emlrtRSI;
  compute_rotation_mat(&st, dv, *(real_T(*)[9]) & y_all[177], ck0);
  st.site = &f_emlrtRSI;
  compute_rotation_mat(&st, dv, *(real_T(*)[9]) & y_all[198], ck1);
  for (i = 0; i < 3; i++) {
    y_true_tmp = ck1[i];
    y = ck1[i + 3];
    absx = ck1[i + 6];
    for (step = 0; step < 3; step++) {
      Psik[i + 3 * step] =
          (y_true_tmp * ck0[step] + y * ck0[step + 3]) + absx * ck0[step + 6];
    }
  }
  st.site = &g_emlrtRSI;
  rot2axis(&st, Psik, w_k_true, &y_true_tmp);
  st.site = &h_emlrtRSI;
  axis2rot(&st, w_k_true, y_true_tmp, ck0);
  st.site = &i_emlrtRSI;
  for (i = 0; i < 9; i++) {
    ck0[i] -= Psik[i];
  }
  b_st.site = &vb_emlrtRSI;
  y = 0.0;
  for (step = 0; step < 3; step++) {
    absx = muDoubleScalarAbs(ck0[3 * step]);
    if (muDoubleScalarIsNaN(absx) || (absx > y)) {
      y = absx;
    }
    absx = muDoubleScalarAbs(ck0[3 * step + 1]);
    if (muDoubleScalarIsNaN(absx) || (absx > y)) {
      y = absx;
    }
    absx = muDoubleScalarAbs(ck0[3 * step + 2]);
    if (muDoubleScalarIsNaN(absx) || (absx > y)) {
      y = absx;
    }
  }
  if ((!muDoubleScalarIsInf(y)) && (!muDoubleScalarIsNaN(y))) {
    c_st.site = &wb_emlrtRSI;
    d_st.site = &xb_emlrtRSI;
    e_st.site = &yb_emlrtRSI;
    f_st.site = &ac_emlrtRSI;
    xzsvdc(&f_st, ck0, s);
    y = s[0];
  }
  if (covrtLogIf(&emlrtCoverageInstance, 0U, 0U, 0, y > 1.0E-6)) {
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 3U);
    y_true_tmp = -y_true_tmp;
  }
  covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 4U);
  w_k_true[0] = w_k_true[0] * y_true_tmp / 0.0001;
  w_k_true[1] = w_k_true[1] * y_true_tmp / 0.0001;
  w_k_true[2] = w_k_true[2] * y_true_tmp / 0.0001;
  st.site = &j_emlrtRSI;
  rot2axis(&st, ck1, phi_true, &y_true_tmp);
  phi_true[0] *= y_true_tmp;
  phi_true[1] *= y_true_tmp;
  phi_true[2] *= y_true_tmp;
  /* current orientation */
}

/* End of code generation (bit_one_step.c) */
