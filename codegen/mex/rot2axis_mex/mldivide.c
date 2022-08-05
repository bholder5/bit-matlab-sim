/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 */

/* Include files */
#include "mldivide.h"
#include "eml_int_forloop_overflow_check.h"
#include "rot2axis_mex_data.h"
#include "rt_nonfinite.h"
#include "warning.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo eb_emlrtRSI = {
    20,         /* lineNo */
    "mldivide", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/lib/matlab/ops/mldivide.m" /* pathName
                                                                      */
};

static emlrtRSInfo fb_emlrtRSI = {
    42,      /* lineNo */
    "mldiv", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/lib/matlab/ops/mldivide.m" /* pathName
                                                                      */
};

static emlrtRSInfo gb_emlrtRSI = {
    67,        /* lineNo */
    "lusolve", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo hb_emlrtRSI = {
    112,          /* lineNo */
    "lusolveNxN", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo ib_emlrtRSI = {
    109,          /* lineNo */
    "lusolveNxN", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo jb_emlrtRSI = {
    124,          /* lineNo */
    "InvAtimesX", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo kb_emlrtRSI = {
    26,        /* lineNo */
    "xgetrfs", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "xgetrfs.m" /* pathName */
};

static emlrtRSInfo lb_emlrtRSI =
    {
        30,       /* lineNo */
        "xgetrf", /* fcnName */
        "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgetrf.m" /* pathName */
};

static emlrtRSInfo mb_emlrtRSI = {
    50,        /* lineNo */
    "xzgetrf", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzgetrf.m" /* pathName */
};

static emlrtRSInfo nb_emlrtRSI = {
    58,        /* lineNo */
    "xzgetrf", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzgetrf.m" /* pathName */
};

static emlrtRSInfo ob_emlrtRSI =
    {
        45,      /* lineNo */
        "xgeru", /* fcnName */
        "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+blas/"
        "xgeru.m" /* pathName */
};

static emlrtRSInfo
    pb_emlrtRSI =
        {
            45,     /* lineNo */
            "xger", /* fcnName */
            "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+blas/"
            "xger.m" /* pathName */
};

static emlrtRSInfo qb_emlrtRSI =
    {
        15,     /* lineNo */
        "xger", /* fcnName */
        "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+refblas/"
        "xger.m" /* pathName */
};

static emlrtRSInfo rb_emlrtRSI =
    {
        54,      /* lineNo */
        "xgerx", /* fcnName */
        "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+refblas/"
        "xgerx.m" /* pathName */
};

static emlrtRSInfo sb_emlrtRSI = {
    90,              /* lineNo */
    "warn_singular", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

/* Function Definitions */
void mldivide(const emlrtStack *sp, const real_T A[81], real_T B[9])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack j_st;
  emlrtStack k_st;
  emlrtStack l_st;
  emlrtStack m_st;
  emlrtStack st;
  real_T b_A[81];
  real_T s;
  real_T smax;
  int32_T b_tmp;
  int32_T info;
  int32_T j;
  int32_T jA;
  int32_T jp1j;
  int32_T k;
  int32_T n;
  int32_T temp_tmp;
  int8_T ipiv[9];
  int8_T i;
  boolean_T overflow;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &eb_emlrtRSI;
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
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  j_st.prev = &i_st;
  j_st.tls = i_st.tls;
  k_st.prev = &j_st;
  k_st.tls = j_st.tls;
  l_st.prev = &k_st;
  l_st.tls = k_st.tls;
  m_st.prev = &l_st;
  m_st.tls = l_st.tls;
  b_st.site = &fb_emlrtRSI;
  c_st.site = &gb_emlrtRSI;
  d_st.site = &ib_emlrtRSI;
  e_st.site = &jb_emlrtRSI;
  f_st.site = &kb_emlrtRSI;
  memcpy(&b_A[0], &A[0], 81U * sizeof(real_T));
  g_st.site = &lb_emlrtRSI;
  for (temp_tmp = 0; temp_tmp < 9; temp_tmp++) {
    ipiv[temp_tmp] = (int8_T)(temp_tmp + 1);
  }
  info = 0;
  for (j = 0; j < 8; j++) {
    b_tmp = j * 10;
    jp1j = b_tmp + 2;
    n = 9 - j;
    jA = 0;
    smax = muDoubleScalarAbs(b_A[b_tmp]);
    for (k = 2; k <= n; k++) {
      s = muDoubleScalarAbs(b_A[(b_tmp + k) - 1]);
      if (s > smax) {
        jA = k - 1;
        smax = s;
      }
    }
    if (b_A[b_tmp + jA] != 0.0) {
      if (jA != 0) {
        jA += j;
        ipiv[j] = (int8_T)(jA + 1);
        for (k = 0; k < 9; k++) {
          temp_tmp = j + k * 9;
          smax = b_A[temp_tmp];
          n = jA + k * 9;
          b_A[temp_tmp] = b_A[n];
          b_A[n] = smax;
        }
      }
      temp_tmp = (b_tmp - j) + 9;
      h_st.site = &mb_emlrtRSI;
      for (n = jp1j; n <= temp_tmp; n++) {
        b_A[n - 1] /= b_A[b_tmp];
      }
    } else {
      info = j + 1;
    }
    n = 7 - j;
    h_st.site = &nb_emlrtRSI;
    i_st.site = &ob_emlrtRSI;
    j_st.site = &pb_emlrtRSI;
    k_st.site = &qb_emlrtRSI;
    jA = b_tmp + 11;
    for (jp1j = 0; jp1j <= n; jp1j++) {
      smax = b_A[(b_tmp + jp1j * 9) + 9];
      if (smax != 0.0) {
        temp_tmp = (jA - j) + 7;
        l_st.site = &rb_emlrtRSI;
        overflow = ((jA <= temp_tmp) && (temp_tmp > 2147483646));
        if (overflow) {
          m_st.site = &bb_emlrtRSI;
          check_forloop_overflow_error(&m_st);
        }
        for (k = jA; k <= temp_tmp; k++) {
          b_A[k - 1] += b_A[((b_tmp + k) - jA) + 1] * -smax;
        }
      }
      jA += 9;
    }
  }
  if ((info == 0) && (!(b_A[80] != 0.0))) {
    info = 9;
  }
  for (n = 0; n < 8; n++) {
    i = ipiv[n];
    if (i != n + 1) {
      smax = B[n];
      B[n] = B[i - 1];
      B[i - 1] = smax;
    }
  }
  for (k = 0; k < 9; k++) {
    jA = 9 * k;
    if (B[k] != 0.0) {
      temp_tmp = k + 2;
      for (n = temp_tmp; n < 10; n++) {
        B[n - 1] -= B[k] * b_A[(n + jA) - 1];
      }
    }
  }
  for (k = 8; k >= 0; k--) {
    jA = 9 * k;
    if (B[k] != 0.0) {
      B[k] /= b_A[k + jA];
      for (n = 0; n < k; n++) {
        B[n] -= B[k] * b_A[n + jA];
      }
    }
  }
  if (info > 0) {
    d_st.site = &hb_emlrtRSI;
    e_st.site = &sb_emlrtRSI;
    warning(&e_st);
  }
}

/* End of code generation (mldivide.c) */
