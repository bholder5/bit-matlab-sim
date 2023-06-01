/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rot2axis_C.c
 *
 * Code generation for function 'rot2axis_C'
 *
 */

/* Include files */
#include "rot2axis_C.h"
#include "bit_one_step_mex_data.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>

/* Variable Definitions */
static emlrtRSInfo yc_emlrtRSI = {
    2,                                          /* lineNo */
    "rot2axis_C",                               /* fcnName */
    "/home/bholder/bit-matlab-sim/rot2axis_C.m" /* pathName */
};

static emlrtRSInfo ad_emlrtRSI = {
    6,                                          /* lineNo */
    "rot2axis_C",                               /* fcnName */
    "/home/bholder/bit-matlab-sim/rot2axis_C.m" /* pathName */
};

static emlrtRTEInfo b_emlrtRTEI = {
    13,                                                            /* lineNo */
    9,                                                             /* colNo */
    "sqrt",                                                        /* fName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/lib/matlab/elfun/sqrt.m" /* pName */
};

static emlrtRTEInfo c_emlrtRTEI = {
    14,                                                            /* lineNo */
    9,                                                             /* colNo */
    "acos",                                                        /* fName */
    "/usr/local/MATLAB/R2023a/toolbox/eml/lib/matlab/elfun/acos.m" /* pName */
};

/* Function Definitions */
/*
 * function [v, phi] = rot2axisC(C)
 */
void rot2axis_C(const emlrtStack *sp, const real_T C[9], real_T v[3],
                real_T *phi)
{
  __m128d r;
  emlrtStack st;
  real_T b_v;
  st.prev = sp;
  st.tls = sp->tls;
  covrtLogFcn(&emlrtCoverageInstance, 19U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 19U, 0U);
  /* 'rot2axis_C:2' phi = acos((C(1,1) + C(2,2) + C(3,3) - 1)/2); */
  st.site = &yc_emlrtRSI;
  *phi = (((C[0] + C[4]) + C[8]) - 1.0) / 2.0;
  if ((*phi < -1.0) || (*phi > 1.0)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &c_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "acos");
  }
  *phi = muDoubleScalarAcos(*phi);
  /* 'rot2axis_C:4' sinphi = sin(phi); */
  /* 'rot2axis_C:5' v = [C(3,2) - C(2,3); C(1,3) - C(3,1); C(2,1) -
   * C(1,2)]/(2*sinphi); */
  b_v = 2.0 * muDoubleScalarSin(*phi);
  v[0] = (C[5] - C[7]) / b_v;
  v[1] = (C[6] - C[2]) / b_v;
  v[2] = (C[1] - C[3]) / b_v;
  /* 'rot2axis_C:6' v = v/sqrt(v'*v); */
  b_v = (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2];
  st.site = &ad_emlrtRSI;
  if (b_v < 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &st, &b_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
        "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
  }
  b_v = muDoubleScalarSqrt(b_v);
  r = _mm_loadu_pd(&v[0]);
  _mm_storeu_pd(&v[0], _mm_div_pd(r, _mm_set1_pd(b_v)));
  v[2] /= b_v;
}

/* End of code generation (rot2axis_C.c) */
