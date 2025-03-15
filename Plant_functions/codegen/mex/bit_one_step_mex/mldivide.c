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
#include "bit_one_step_mex_data.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ub_emlrtRSI = {
    20,                               /* lineNo */
    "eml_int_forloop_overflow_check", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/lib/matlab/eml/"
    "eml_int_forloop_overflow_check.m" /* pathName */
};

static emlrtRSInfo vb_emlrtRSI = {
    20,         /* lineNo */
    "mldivide", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/lib/matlab/ops/mldivide.m" /* pathName
                                                                      */
};

static emlrtRSInfo wb_emlrtRSI = {
    42,      /* lineNo */
    "mldiv", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/lib/matlab/ops/mldivide.m" /* pathName
                                                                      */
};

static emlrtRSInfo xb_emlrtRSI = {
    67,        /* lineNo */
    "lusolve", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo yb_emlrtRSI = {
    109,          /* lineNo */
    "lusolveNxN", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo ac_emlrtRSI = {
    112,          /* lineNo */
    "lusolveNxN", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo bc_emlrtRSI = {
    124,          /* lineNo */
    "InvAtimesX", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/lusolve.m" /* pathName
                                                                           */
};

static emlrtRSInfo cc_emlrtRSI = {
    26,        /* lineNo */
    "xgetrfs", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+lapack/"
    "xgetrfs.m" /* pathName */
};

static emlrtRSInfo dc_emlrtRSI = {
    27,        /* lineNo */
    "xgetrfs", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+lapack/"
    "xgetrfs.m" /* pathName */
};

static emlrtRSInfo ec_emlrtRSI =
    {
        31,       /* lineNo */
        "xgetrf", /* fcnName */
        "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgetrf.m" /* pathName */
};

static emlrtRSInfo fc_emlrtRSI = {
    36,        /* lineNo */
    "xzgetrf", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzgetrf.m" /* pathName */
};

static emlrtRSInfo gc_emlrtRSI = {
    50,        /* lineNo */
    "xzgetrf", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzgetrf.m" /* pathName */
};

static emlrtRSInfo hc_emlrtRSI = {
    58,        /* lineNo */
    "xzgetrf", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzgetrf.m" /* pathName */
};

static emlrtRSInfo ic_emlrtRSI =
    {
        23,       /* lineNo */
        "ixamax", /* fcnName */
        "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+blas/"
        "ixamax.m" /* pathName */
};

static emlrtRSInfo jc_emlrtRSI = {
    24,       /* lineNo */
    "ixamax", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+refblas/"
    "ixamax.m" /* pathName */
};

static emlrtRSInfo kc_emlrtRSI =
    {
        45,      /* lineNo */
        "xgeru", /* fcnName */
        "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+blas/"
        "xgeru.m" /* pathName */
};

static emlrtRSInfo
    lc_emlrtRSI =
        {
            45,     /* lineNo */
            "xger", /* fcnName */
            "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+blas/"
            "xger.m" /* pathName */
};

static emlrtRSInfo mc_emlrtRSI =
    {
        15,     /* lineNo */
        "xger", /* fcnName */
        "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+refblas/"
        "xger.m" /* pathName */
};

static emlrtRSInfo nc_emlrtRSI =
    {
        41,      /* lineNo */
        "xgerx", /* fcnName */
        "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+refblas/"
        "xgerx.m" /* pathName */
};

static emlrtRSInfo oc_emlrtRSI =
    {
        54,      /* lineNo */
        "xgerx", /* fcnName */
        "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+refblas/"
        "xgerx.m" /* pathName */
};

static emlrtRSInfo pc_emlrtRSI =
    {
        18,       /* lineNo */
        "xgetrs", /* fcnName */
        "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xgetrs.m" /* pathName */
};

static emlrtRSInfo qc_emlrtRSI = {
    32,        /* lineNo */
    "xzgetrs", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzgetrs.m" /* pathName */
};

static emlrtRSInfo rc_emlrtRSI = {
    36,        /* lineNo */
    "xzgetrs", /* fcnName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzgetrs.m" /* pathName */
};

static emlrtRSInfo sc_emlrtRSI =
    {
        59,      /* lineNo */
        "xtrsm", /* fcnName */
        "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+blas/"
        "xtrsm.m" /* pathName */
};

static emlrtRSInfo tc_emlrtRSI =
    {
        71,      /* lineNo */
        "xtrsm", /* fcnName */
        "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+refblas/"
        "xtrsm.m" /* pathName */
};

static emlrtRSInfo uc_emlrtRSI =
    {
        51,      /* lineNo */
        "xtrsm", /* fcnName */
        "/usr/local/MATLAB/R2023a/toolbox/eml/eml/+coder/+internal/+refblas/"
        "xtrsm.m" /* pathName */
};

/* Function Definitions */
/*
 *
 */
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
  real_T smax;
  int32_T info;
  int32_T j;
  int32_T jA;
  int32_T jp1j;
  int32_T k;
  int32_T n;
  int32_T temp_tmp;
  int8_T ipiv[9];
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &vb_emlrtRSI;
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
  b_st.site = &wb_emlrtRSI;
  c_st.site = &xb_emlrtRSI;
  d_st.site = &yb_emlrtRSI;
  e_st.site = &bc_emlrtRSI;
  f_st.site = &cc_emlrtRSI;
  memcpy(&b_A[0], &A[0], 81U * sizeof(real_T));
  g_st.site = &ec_emlrtRSI;
  for (temp_tmp = 0; temp_tmp < 9; temp_tmp++) {
    ipiv[temp_tmp] = (int8_T)(temp_tmp + 1);
  }
  info = 0;
  for (j = 0; j < 8; j++) {
    int32_T b_tmp;
    b_tmp = j * 10;
    jp1j = b_tmp + 2;
    n = 9 - j;
    h_st.site = &fc_emlrtRSI;
    i_st.site = &ic_emlrtRSI;
    jA = 0;
    smax = muDoubleScalarAbs(b_A[b_tmp]);
    j_st.site = &jc_emlrtRSI;
    for (k = 2; k <= n; k++) {
      real_T s;
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
      h_st.site = &gc_emlrtRSI;
      for (n = jp1j; n <= temp_tmp; n++) {
        b_A[n - 1] /= b_A[b_tmp];
      }
    } else {
      info = j + 1;
    }
    n = 7 - j;
    h_st.site = &hc_emlrtRSI;
    i_st.site = &kc_emlrtRSI;
    j_st.site = &lc_emlrtRSI;
    k_st.site = &mc_emlrtRSI;
    jA = b_tmp + 11;
    l_st.site = &nc_emlrtRSI;
    for (jp1j = 0; jp1j <= n; jp1j++) {
      smax = b_A[(b_tmp + jp1j * 9) + 9];
      if (smax != 0.0) {
        temp_tmp = (jA - j) + 7;
        l_st.site = &oc_emlrtRSI;
        if ((jA <= temp_tmp) && (temp_tmp > 2147483646)) {
          m_st.site = &ub_emlrtRSI;
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
  f_st.site = &dc_emlrtRSI;
  g_st.site = &pc_emlrtRSI;
  for (n = 0; n < 8; n++) {
    int8_T i;
    i = ipiv[n];
    if (i != n + 1) {
      smax = B[n];
      B[n] = B[i - 1];
      B[i - 1] = smax;
    }
  }
  h_st.site = &qc_emlrtRSI;
  i_st.site = &sc_emlrtRSI;
  for (k = 0; k < 9; k++) {
    jA = 9 * k;
    if (B[k] != 0.0) {
      temp_tmp = k + 2;
      j_st.site = &tc_emlrtRSI;
      for (n = temp_tmp; n < 10; n++) {
        B[n - 1] -= B[k] * b_A[(n + jA) - 1];
      }
    }
  }
  h_st.site = &rc_emlrtRSI;
  i_st.site = &sc_emlrtRSI;
  for (k = 8; k >= 0; k--) {
    jA = 9 * k;
    smax = B[k];
    if (smax != 0.0) {
      smax /= b_A[k + jA];
      B[k] = smax;
      j_st.site = &uc_emlrtRSI;
      for (n = 0; n < k; n++) {
        B[n] -= B[k] * b_A[n + jA];
      }
    }
  }
  if (info > 0) {
    d_st.site = &ac_emlrtRSI;
  }
}

/* End of code generation (mldivide.c) */
