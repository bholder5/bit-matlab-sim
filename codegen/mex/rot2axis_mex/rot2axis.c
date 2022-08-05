/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rot2axis.c
 *
 * Code generation for function 'rot2axis'
 *
 */

/* Include files */
#include "rot2axis.h"
#include "rot2axis_mex_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo ub_emlrtRSI = {
    2,                                                   /* lineNo */
    "rot2axis",                                          /* fcnName */
    "/home/brad/bit-matlab-sim/Miscellaneous/rot2axis.m" /* pathName */
};

static emlrtRTEInfo emlrtRTEI = {
    14,                                                            /* lineNo */
    9,                                                             /* colNo */
    "acos",                                                        /* fName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/lib/matlab/elfun/acos.m" /* pName */
};

/* Function Definitions */
void rot2axis(const emlrtStack *sp, const real_T C[9], real_T v[3], real_T *phi)
{
  emlrtStack st;
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  st.prev = sp;
  st.tls = sp->tls;
  covrtLogFcn(&emlrtCoverageInstance, 9U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 0U);
  st.site = &ub_emlrtRSI;
  *phi = (((C[0] + C[4]) + C[8]) - 1.0) / 2.0;
  if ((*phi < -1.0) || (*phi > 1.0)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "acos");
  }
  *phi = muDoubleScalarAcos(*phi);
  scale = 2.0 * muDoubleScalarSin(*phi);
  v[0] = (C[5] - C[7]) / scale;
  v[1] = (C[6] - C[2]) / scale;
  v[2] = (C[1] - C[3]) / scale;
  scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(v[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = muDoubleScalarAbs(v[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = muDoubleScalarAbs(v[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * muDoubleScalarSqrt(y);
  v[0] /= y;
  v[1] /= y;
  v[2] /= y;
}

/* End of code generation (rot2axis.c) */
