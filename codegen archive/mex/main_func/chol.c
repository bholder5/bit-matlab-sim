/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 *
 * chol.c
 *
 * Code generation for function 'chol'
 *
 */

/* Include files */
#include "chol.h"
#include "main_func_data.h"
#include "rt_nonfinite.h"
#include "lapacke.h"
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo db_emlrtRSI = {
    84,     /* lineNo */
    "chol", /* fcnName */
    "/usr/local/MATLAB/R2022a/toolbox/eml/eml/+coder/+internal/chol.m" /* pathName
                                                                        */
};

static emlrtRSInfo eb_emlrtRSI = {
    101,    /* lineNo */
    "chol", /* fcnName */
    "/usr/local/MATLAB/R2022a/toolbox/eml/eml/+coder/+internal/chol.m" /* pathName
                                                                        */
};

static emlrtRSInfo fb_emlrtRSI =
    {
        79,             /* lineNo */
        "ceval_xpotrf", /* fcnName */
        "/usr/local/MATLAB/R2022a/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xpotrf.m" /* pathName */
};

static emlrtRSInfo gb_emlrtRSI =
    {
        13,       /* lineNo */
        "xpotrf", /* fcnName */
        "/usr/local/MATLAB/R2022a/toolbox/eml/eml/+coder/+internal/+lapack/"
        "xpotrf.m" /* pathName */
};

static emlrtRTEInfo b_emlrtRTEI = {
    47,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "/usr/local/MATLAB/R2022a/toolbox/eml/eml/+coder/+internal/+lapack/"
    "infocheck.m" /* pName */
};

static emlrtRTEInfo c_emlrtRTEI = {
    44,          /* lineNo */
    13,          /* colNo */
    "infocheck", /* fName */
    "/usr/local/MATLAB/R2022a/toolbox/eml/eml/+coder/+internal/+lapack/"
    "infocheck.m" /* pName */
};

static emlrtRTEInfo d_emlrtRTEI = {
    109,    /* lineNo */
    27,     /* colNo */
    "chol", /* fName */
    "/usr/local/MATLAB/R2022a/toolbox/eml/eml/+coder/+internal/chol.m" /* pName
                                                                        */
};

/* Function Definitions */
void chol(const emlrtStack *sp, real_T A[81])
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
  int32_T jmax;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &db_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &gb_emlrtRSI;
  info_t = LAPACKE_dpotrf_work(102, 'U', (ptrdiff_t)9, &A[0], (ptrdiff_t)9);
  info = (int32_T)info_t;
  c_st.site = &fb_emlrtRSI;
  if (info < 0) {
    if (info == -1010) {
      emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI, "MATLAB:nomem",
                                    "MATLAB:nomem", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(
          &c_st, &b_emlrtRTEI, "Coder:toolbox:LAPACKCallErrorInfo",
          "Coder:toolbox:LAPACKCallErrorInfo", 5, 4, 19, &fname[0], 12, info);
    }
  }
  if (info == 0) {
    jmax = 7;
  } else {
    jmax = info - 3;
  }
  for (j = 0; j <= jmax; j++) {
    int32_T a;
    a = j + 2;
    st.site = &eb_emlrtRSI;
    if (a <= jmax + 2) {
      memset(&A[(j * 9 + a) + -1], 0, ((jmax - a) + 3) * sizeof(real_T));
    }
  }
  if (info != 0) {
    emlrtErrorWithMessageIdR2018a(sp, &d_emlrtRTEI, "MATLAB:posdef",
                                  "MATLAB:posdef", 0);
  }
}

/* End of code generation (chol.c) */
