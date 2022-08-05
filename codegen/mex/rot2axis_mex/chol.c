/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * chol.c
 *
 * Code generation for function 'chol'
 *
 */

/* Include files */
#include "chol.h"
#include "rot2axis_mex_data.h"
#include "rt_nonfinite.h"
#include "lapacke.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo w_emlrtRSI = {
    74,         /* lineNo */
    "cholesky", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/lib/matlab/matfun/chol.m" /* pathName
                                                                     */
};

static emlrtRSInfo x_emlrtRSI = {
    92,         /* lineNo */
    "cholesky", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/lib/matlab/matfun/chol.m" /* pathName
                                                                     */
};

static emlrtRSInfo y_emlrtRSI =
    {
        79,             /* lineNo */
        "ceval_xpotrf", /* fcnName */
        "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xpotrf.m" /* pathName */
};

static emlrtRSInfo ab_emlrtRSI =
    {
        13,       /* lineNo */
        "xpotrf", /* fcnName */
        "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xpotrf.m" /* pathName */
};

static emlrtRTEInfo e_emlrtRTEI = {
    47,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "infocheck.m" /* pName */
};

static emlrtRTEInfo f_emlrtRTEI = {
    44,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+lapack/"
    "infocheck.m" /* pName */
};

static emlrtRTEInfo g_emlrtRTEI = {
    80,                                                             /* lineNo */
    23,                                                             /* colNo */
    "cholesky",                                                     /* fName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/lib/matlab/matfun/chol.m" /* pName */
};

/* Function Definitions */
void cholesky(const emlrtStack *sp, real_T A[81])
{
  static const char_T fname[19] = {'L', 'A', 'P', 'A', 'C', 'K', 'E',
                                   '_', 'd', 'p', 'o', 't', 'r', 'f',
                                   '_', 'w', 'o', 'r', 'k'};
  ptrdiff_t info_t;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T info;
  int32_T j;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &w_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &ab_emlrtRSI;
  info_t = LAPACKE_dpotrf_work(102, 'U', (ptrdiff_t)9, &A[0], (ptrdiff_t)9);
  info = (int32_T)info_t;
  c_st.site = &y_emlrtRSI;
  if (info < 0) {
    if (info == -1010) {
      emlrtErrorWithMessageIdR2018a(&c_st, &f_emlrtRTEI, "MATLAB:nomem",
                                    "MATLAB:nomem", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &e_emlrtRTEI, "Coder:toolbox:LAPACKCallErrorInfo",
          "Coder:toolbox:LAPACKCallErrorInfo", 5, 4, 19, &fname[0], 12, info);
    }
  }
  if (info != 0) {
    emlrtErrorWithMessageIdR2018a(sp, &g_emlrtRTEI, "Coder:MATLAB:posdef",
                                  "Coder:MATLAB:posdef", 0);
  }
  for (j = 0; j < 9; j++) {
    info = j + 2;
    st.site = &x_emlrtRSI;
    if (info <= 9) {
      memset(&A[(j * 9 + info) + -1], 0, (-info + 10) * sizeof(real_T));
    }
  }
}

/* End of code generation (chol.c) */
