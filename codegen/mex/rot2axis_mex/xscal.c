/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xscal.c
 *
 * Code generation for function 'xscal'
 *
 */

/* Include files */
#include "xscal.h"
#include "eml_int_forloop_overflow_check.h"
#include "rot2axis_mex_data.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo cb_emlrtRSI =
    {
        31,      /* lineNo */
        "xscal", /* fcnName */
        "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+blas/"
        "xscal.m" /* pathName */
};

static emlrtRSInfo db_emlrtRSI =
    {
        18,      /* lineNo */
        "xscal", /* fcnName */
        "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+refblas/"
        "xscal.m" /* pathName */
};

/* Function Definitions */
void b_xscal(real_T a, real_T x[3])
{
  int32_T k;
  for (k = 2; k < 4; k++) {
    x[k - 1] *= a;
  }
}

void xscal(const emlrtStack *sp, int32_T n, real_T a, real_T x[9], int32_T ix0)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T b;
  int32_T k;
  boolean_T overflow;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &cb_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b = (ix0 + n) - 1;
  b_st.site = &db_emlrtRSI;
  overflow = ((ix0 <= b) && (b > 2147483646));
  if (overflow) {
    c_st.site = &bb_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }
  for (k = ix0; k <= b; k++) {
    x[k - 1] *= a;
  }
}

/* End of code generation (xscal.c) */
