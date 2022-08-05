/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xzsvdc.c
 *
 * Code generation for function 'xzsvdc'
 *
 */

/* Include files */
#include "xzsvdc.h"
#include "rot2axis_mex_data.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xscal.h"
#include "blas.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo bc_emlrtRSI = {
    407,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo cc_emlrtRSI = {
    394,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo dc_emlrtRSI = {
    391,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo ec_emlrtRSI = {
    380,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo fc_emlrtRSI = {
    353,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo gc_emlrtRSI = {
    351,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo hc_emlrtRSI = {
    334,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo ic_emlrtRSI = {
    120,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo jc_emlrtRSI = {
    114,      /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo kc_emlrtRSI = {
    82,       /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo lc_emlrtRSI = {
    77,       /* lineNo */
    "xzsvdc", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pathName */
};

static emlrtRSInfo oc_emlrtRSI = {
    21,                   /* lineNo */
    "scaleVectorByRecip", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/"
    "scaleVectorByRecip.m" /* pathName */
};

static emlrtRTEInfo b_emlrtRTEI = {
    269,      /* lineNo */
    13,       /* colNo */
    "xzsvdc", /* fName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/+reflapack/"
    "xzsvdc.m" /* pName */
};

static emlrtRTEInfo d_emlrtRTEI = {
    13,                                                            /* lineNo */
    9,                                                             /* colNo */
    "sqrt",                                                        /* fName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/lib/matlab/elfun/sqrt.m" /* pName */
};

/* Function Definitions */
void xzsvdc(const emlrtStack *sp, real_T A[9], real_T S[3])
{
  emlrtStack b_st;
  emlrtStack st;
  real_T e[3];
  real_T s[3];
  real_T work[3];
  real_T b;
  real_T nrm;
  real_T r;
  real_T scale;
  real_T sm;
  real_T sn;
  real_T snorm;
  real_T sqds;
  int32_T exitg2;
  int32_T k;
  int32_T m;
  int32_T q;
  int32_T qjj;
  int32_T qp1;
  int32_T qq;
  boolean_T apply_transform;
  boolean_T exitg1;
  boolean_T exitg3;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  s[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  s[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  s[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  for (q = 0; q < 2; q++) {
    qp1 = q + 2;
    qq = (q + 3 * q) + 1;
    apply_transform = false;
    st.site = &lc_emlrtRSI;
    nrm = xnrm2(&st, 3 - q, A, qq);
    if (nrm > 0.0) {
      apply_transform = true;
      if (A[qq - 1] < 0.0) {
        sqds = -nrm;
        s[q] = -nrm;
      } else {
        sqds = nrm;
        s[q] = nrm;
      }
      st.site = &kc_emlrtRSI;
      if (muDoubleScalarAbs(sqds) >= 1.0020841800044864E-292) {
        b_st.site = &oc_emlrtRSI;
        xscal(&b_st, 3 - q, 1.0 / sqds, A, qq);
      } else {
        qjj = (qq - q) + 2;
        for (k = qq; k <= qjj; k++) {
          A[k - 1] /= s[q];
        }
      }
      A[qq - 1]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0;
    }
    for (k = qp1; k < 4; k++) {
      qjj = q + 3 * (k - 1);
      if (apply_transform) {
        xaxpy(3 - q, -(xdotc(3 - q, A, qq, A, qjj + 1) / A[q + 3 * q]), qq, A,
              qjj + 1);
      }
      e[k - 1] = A[qjj];
    }
    if (q + 1 <= 1) {
      st.site = &jc_emlrtRSI;
      nrm = b_xnrm2(e);
      if (nrm == 0.0) {
        e[0] = 0.0;
      } else {
        if (e[1] < 0.0) {
          e[0] = -nrm;
        } else {
          e[0] = nrm;
        }
        st.site = &ic_emlrtRSI;
        sm = e[0];
        if (muDoubleScalarAbs(e[0]) >= 1.0020841800044864E-292) {
          b_xscal(1.0 / e[0], e);
        } else {
          for (k = qp1; k < 4; k++) {
            e[k - 1] /= sm;
          }
        }
        e[1]++;
        e[0] = -e[0];
        for (k = qp1; k < 4; k++) {
          work[k - 1] = 0.0;
        }
        for (k = qp1; k < 4; k++) {
          b_xaxpy(e[k - 1], A, 3 * (k - 1) + 2, work);
        }
        for (k = qp1; k < 4; k++) {
          c_xaxpy(-e[k - 1] / e[1], work, A, 3 * (k - 1) + 2);
        }
      }
    }
  }
  m = 1;
  s[2] = A[8];
  e[1] = A[7];
  e[2] = 0.0;
  qp1 = 0;
  sqds = s[0];
  if (s[0] != 0.0) {
    nrm = muDoubleScalarAbs(s[0]);
    r = s[0] / nrm;
    sqds = nrm;
    s[0] = nrm;
    e[0] /= r;
  }
  if (e[0] != 0.0) {
    nrm = muDoubleScalarAbs(e[0]);
    r = nrm / e[0];
    e[0] = nrm;
    s[1] *= r;
  }
  snorm = muDoubleScalarMax(muDoubleScalarAbs(sqds), e[0]);
  sqds = s[1];
  if (s[1] != 0.0) {
    nrm = muDoubleScalarAbs(s[1]);
    r = s[1] / nrm;
    sqds = nrm;
    s[1] = nrm;
    e[1] = A[7] / r;
  }
  if (e[1] != 0.0) {
    nrm = muDoubleScalarAbs(e[1]);
    r = nrm / e[1];
    e[1] = nrm;
    s[2] = A[8] * r;
  }
  snorm = muDoubleScalarMax(snorm,
                            muDoubleScalarMax(muDoubleScalarAbs(sqds), e[1]));
  sqds = s[2];
  if (s[2] != 0.0) {
    nrm = muDoubleScalarAbs(s[2]);
    sqds = nrm;
    s[2] = nrm;
  }
  snorm =
      muDoubleScalarMax(snorm, muDoubleScalarMax(muDoubleScalarAbs(sqds), 0.0));
  exitg1 = false;
  while ((!exitg1) && (m + 2 > 0)) {
    if (qp1 >= 75) {
      emlrtErrorWithMessageIdR2018a(sp, &b_emlrtRTEI,
                                    "Coder:MATLAB:svd_NoConvergence",
                                    "Coder:MATLAB:svd_NoConvergence", 0);
    } else {
      k = m;
      do {
        exitg2 = 0;
        q = k + 1;
        if (k + 1 == 0) {
          exitg2 = 1;
        } else {
          nrm = muDoubleScalarAbs(e[k]);
          if ((nrm <= 2.2204460492503131E-16 * (muDoubleScalarAbs(s[k]) +
                                                muDoubleScalarAbs(s[k + 1]))) ||
              (nrm <= 1.0020841800044864E-292) ||
              ((qp1 > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
            e[k] = 0.0;
            exitg2 = 1;
          } else {
            k--;
          }
        }
      } while (exitg2 == 0);
      if (k + 1 == m + 1) {
        qjj = 4;
      } else {
        qq = m + 2;
        qjj = m + 2;
        exitg3 = false;
        while ((!exitg3) && (qjj >= k + 1)) {
          qq = qjj;
          if (qjj == k + 1) {
            exitg3 = true;
          } else {
            nrm = 0.0;
            if (qjj < m + 2) {
              nrm = muDoubleScalarAbs(e[qjj - 1]);
            }
            if (qjj > k + 2) {
              nrm += muDoubleScalarAbs(e[qjj - 2]);
            }
            r = muDoubleScalarAbs(s[qjj - 1]);
            if ((r <= 2.2204460492503131E-16 * nrm) ||
                (r <= 1.0020841800044864E-292)) {
              s[qjj - 1] = 0.0;
              exitg3 = true;
            } else {
              qjj--;
            }
          }
        }
        if (qq == k + 1) {
          qjj = 3;
        } else if (qq == m + 2) {
          qjj = 1;
        } else {
          qjj = 2;
          q = qq;
        }
      }
      switch (qjj) {
      case 1:
        r = e[m];
        e[m] = 0.0;
        qjj = m + 1;
        for (k = qjj; k >= q + 1; k--) {
          st.site = &hc_emlrtRSI;
          scale = 0.0;
          sn = 0.0;
          drotg(&s[k - 1], &r, &scale, &sn);
          if (k > q + 1) {
            r = -sn * e[0];
            e[0] *= scale;
          }
        }
        break;
      case 2:
        r = e[q - 1];
        e[q - 1] = 0.0;
        st.site = &gc_emlrtRSI;
        for (k = q + 1; k <= m + 2; k++) {
          st.site = &fc_emlrtRSI;
          scale = 0.0;
          sn = 0.0;
          drotg(&s[k - 1], &r, &scale, &sn);
          nrm = e[k - 1];
          r = -sn * nrm;
          e[k - 1] = nrm * scale;
        }
        break;
      case 3:
        qjj = m + 1;
        nrm = s[m + 1];
        scale = muDoubleScalarMax(
            muDoubleScalarMax(
                muDoubleScalarMax(muDoubleScalarMax(muDoubleScalarAbs(nrm),
                                                    muDoubleScalarAbs(s[m])),
                                  muDoubleScalarAbs(e[m])),
                muDoubleScalarAbs(s[q])),
            muDoubleScalarAbs(e[q]));
        sm = nrm / scale;
        nrm = s[m] / scale;
        r = e[m] / scale;
        sqds = s[q] / scale;
        b = ((nrm + sm) * (nrm - sm) + r * r) / 2.0;
        nrm = sm * r;
        nrm *= nrm;
        if ((b != 0.0) || (nrm != 0.0)) {
          r = b * b + nrm;
          st.site = &ec_emlrtRSI;
          if (r < 0.0) {
            emlrtErrorWithMessageIdR2018a(
                &st, &d_emlrtRTEI, "Coder:toolbox:ElFunDomainError",
                "Coder:toolbox:ElFunDomainError", 3, 4, 4, "sqrt");
          }
          r = muDoubleScalarSqrt(r);
          if (b < 0.0) {
            r = -r;
          }
          r = nrm / (b + r);
        } else {
          r = 0.0;
        }
        r += (sqds + sm) * (sqds - sm);
        nrm = sqds * (e[q] / scale);
        st.site = &dc_emlrtRSI;
        for (k = q + 1; k <= qjj; k++) {
          st.site = &cc_emlrtRSI;
          scale = 0.0;
          sn = 0.0;
          drotg(&r, &nrm, &scale, &sn);
          if (k > q + 1) {
            e[0] = r;
          }
          nrm = e[k - 1];
          r = s[k - 1];
          sm = scale * r + sn * nrm;
          e[k - 1] = scale * nrm - sn * r;
          sqds = s[k];
          b = sn * sqds;
          sqds *= scale;
          st.site = &bc_emlrtRSI;
          scale = 0.0;
          sn = 0.0;
          drotg(&sm, &b, &scale, &sn);
          s[k - 1] = sm;
          r = scale * e[k - 1] + sn * sqds;
          sqds = -sn * e[k - 1] + scale * sqds;
          s[k] = sqds;
          nrm = sn * e[k];
          e[k] *= scale;
        }
        e[m] = r;
        qp1++;
        break;
      default:
        if (s[q] < 0.0) {
          s[q] = -s[q];
        }
        qp1 = q + 1;
        while ((q + 1 < 3) && (s[q] < s[qp1])) {
          nrm = s[q];
          s[q] = s[qp1];
          s[qp1] = nrm;
          q = qp1;
          qp1++;
        }
        qp1 = 0;
        m--;
        break;
      }
    }
  }
  S[0] = s[0];
  S[1] = s[1];
  S[2] = s[2];
}

/* End of code generation (xzsvdc.c) */
