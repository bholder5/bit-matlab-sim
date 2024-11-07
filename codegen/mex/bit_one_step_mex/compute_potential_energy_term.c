/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_potential_energy_term.c
 *
 * Code generation for function 'compute_potential_energy_term'
 *
 */

/* Include files */
#include "compute_potential_energy_term.h"
#include "axis2rot.h"
#include "bit_one_step_mex_data.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo eb_emlrtRSI = {
    17,                              /* lineNo */
    "compute_potential_energy_term", /* fcnName */
    "/home/bholder/bit-matlab-sim/Plant_functions/"
    "compute_potential_energy_term.m" /* pathName */
};

static emlrtBCInfo f_emlrtBCI = {
    1,                               /* iFirst */
    9,                               /* iLast */
    30,                              /* lineNo */
    29,                              /* colNo */
    "m_n",                           /* aName */
    "compute_potential_energy_term", /* fName */
    "/home/bholder/bit-matlab-sim/Plant_functions/"
    "compute_potential_energy_term.m", /* pName */
    0                                  /* checkKind */
};

/* Function Definitions */
/*
 * function Pot = compute_potential_energy_term(theta, c_n, z_n, m_n, r_n1_n,
 * g0)
 */
void compute_potential_energy_term(const emlrtStack *sp, const real_T theta[9],
                                   const real_T c_n[27], const real_T z_n[27],
                                   const real_T m_n[9], const real_T r_n1_n[27],
                                   const real_T g0[3], real_T Pot[9])
{
  emlrtStack st;
  real_T C_n[81];
  real_T C_n_rate[81];
  real_T b_C_n[9];
  real_T dC_10[9];
  real_T dVdtheta_i[9];
  real_T t1[9];
  real_T C_n_tmp;
  real_T b_C_n_tmp;
  real_T d;
  int32_T i;
  int32_T i1;
  int32_T j;
  int32_T k;
  int32_T n;
  int32_T q;
  st.prev = sp;
  st.tls = sp->tls;
  covrtLogFcn(&emlrtCoverageInstance, 9U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 0U);
  /*  This function is implementing the potential energy term of the */
  /*  lagrangian. See section 3.1.1 of Romualdez Thesis, specifically */
  /*  Eq3.25-3.26. The rate is because of the partial derivitive of V wrt */
  /*  theta.  */
  /* Determine relative rotation matrices and rates for each joint */
  /* 'compute_potential_energy_term:8' ndof = length(theta); */
  /* rotation matricies */
  /* 'compute_potential_energy_term:11' C_n = zeros(3,3,ndof); */
  /* 'compute_potential_energy_term:12' C_n_rate = C_n; */
  /*  Potential energy */
  /* 'compute_potential_energy_term:14' Pot = zeros(ndof,1); */
  memset(&Pot[0], 0, 9U * sizeof(real_T));
  /* 'compute_potential_energy_term:16' for n = 1:ndof */
  b_C_n[0] = -0.0;
  b_C_n[4] = -0.0;
  b_C_n[8] = -0.0;
  for (n = 0; n < 9; n++) {
    covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 1U);
    /* 'compute_potential_energy_term:17' C_n(:,:,n) = axis2rot(z_n(:,n),
     * theta(n)); */
    st.site = &eb_emlrtRSI;
    axis2rot(&st, &z_n[3 * n], theta[n], *(real_T(*)[9]) & C_n[9 * n]);
    /* Partial of Rot_n with respect to theta_n */
    /* 'compute_potential_energy_term:19' C_n_rate(:,:,n) = -1 * xmat(z_n(:,n))
     * * C_n(:,:,n); */
    covrtLogFcn(&emlrtCoverageInstance, 2U, 0U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 2U, 0U);
    /* 'xmat:2' mat = [        0, -vec(3),  vec(2); */
    /* 'xmat:3'             vec(3),         0, -vec(1); */
    /* 'xmat:4'            -vec(2),  vec(1),         0]; */
    C_n_tmp = z_n[3 * n + 2];
    b_C_n[3] = C_n_tmp;
    b_C_n_tmp = z_n[3 * n + 1];
    b_C_n[6] = -b_C_n_tmp;
    b_C_n[1] = -C_n_tmp;
    C_n_tmp = z_n[3 * n];
    b_C_n[7] = C_n_tmp;
    b_C_n[2] = b_C_n_tmp;
    b_C_n[5] = -C_n_tmp;
    for (i = 0; i < 3; i++) {
      C_n_tmp = b_C_n[i];
      b_C_n_tmp = b_C_n[i + 3];
      d = b_C_n[i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        q = 3 * i1 + 9 * n;
        C_n_rate[(i + 3 * i1) + 9 * n] =
            (C_n_tmp * C_n[q] + b_C_n_tmp * C_n[q + 1]) + d * C_n[q + 2];
      }
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 0, 0);
  /*  Compute potential energy term. First loop is cycling through each */
  /*  koints contribution. */
  /* 'compute_potential_energy_term:24' for n = 1:ndof */
  for (n = 0; n < 9; n++) {
    __m128d r;
    __m128d r1;
    real_T m_i;
    covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 1, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 2U);
    /* 'compute_potential_energy_term:25' m_i = 0; */
    m_i = 0.0;
    /* 'compute_potential_energy_term:26' dVdtheta_i = zeros(ndof,1); */
    memset(&dVdtheta_i[0], 0, 9U * sizeof(real_T));
    /*  mass of remaining link (ex. 7 + 8 + 9) is totla mass at OF joint */
    /* 'compute_potential_energy_term:29' for q = n:ndof */
    i = 8 - n;
    for (q = 0; q <= i; q++) {
      covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 2, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 3U);
      /* 'compute_potential_energy_term:30' m_i = m_i + m_n(q); */
      i1 = (n + q) + 1;
      if (i1 > 9) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, 9, &f_emlrtBCI, (emlrtConstCTX)sp);
      }
      m_i += m_n[i1 - 1];
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b((emlrtConstCTX)sp);
      }
    }
    covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 2, 0);
    /* 'compute_potential_energy_term:33' if (m_i ~= 0) */
    if (covrtLogIf(&emlrtCoverageInstance, 9U, 0U, 0, m_i != 0.0)) {
      real_T b_r_n1_n;
      covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 4U);
      /* 'compute_potential_energy_term:34' t1 = zeros(ndof,1); */
      memset(&t1[0], 0, 9U * sizeof(real_T));
      /* 'compute_potential_energy_term:35' t2 = t1; */
      /* Cycling through all joints up to joint n, the jint we are */
      /* currently calculating the contribution from */
      /*  */
      /* terms up to n-1 links.  */
      /* 'compute_potential_energy_term:41' for j = 1:n-1 */
      for (j = 0; j < n; j++) {
        covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 3, 1);
        covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 5U);
        /* 'compute_potential_energy_term:42' dC_10 = eye(3); */
        memset(&dC_10[0], 0, 9U * sizeof(real_T));
        dC_10[0] = 1.0;
        dC_10[4] = 1.0;
        dC_10[8] = 1.0;
        /* 'compute_potential_energy_term:44' for k = 1:n-1 */
        for (k = 0; k < n; k++) {
          covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 4, 1);
          /* This multiplies the rotation matricies successively */
          /*  until the link j where the rate is inserted instead */
          /* 'compute_potential_energy_term:47' if (k == j) */
          if (covrtLogIf(&emlrtCoverageInstance, 9U, 0U, 1, k == j)) {
            covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 6U);
            /* 'compute_potential_energy_term:48' dC_10 = C_n_rate(:,:,k) *
             * dC_10; */
            for (i = 0; i < 3; i++) {
              i1 = i + 9 * k;
              for (q = 0; q < 3; q++) {
                b_C_n[i + 3 * q] = (C_n_rate[i1] * dC_10[3 * q] +
                                    C_n_rate[i1 + 3] * dC_10[3 * q + 1]) +
                                   C_n_rate[i1 + 6] * dC_10[3 * q + 2];
              }
            }
            memcpy(&dC_10[0], &b_C_n[0], 9U * sizeof(real_T));
          } else {
            covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 7U);
            /* 'compute_potential_energy_term:49' else */
            /* 'compute_potential_energy_term:50' dC_10 = C_n(:,:,k) * dC_10; */
            for (i = 0; i < 3; i++) {
              i1 = i + 9 * k;
              for (q = 0; q < 3; q++) {
                b_C_n[i + 3 * q] =
                    (C_n[i1] * dC_10[3 * q] + C_n[i1 + 3] * dC_10[3 * q + 1]) +
                    C_n[i1 + 6] * dC_10[3 * q + 2];
              }
            }
            memcpy(&dC_10[0], &b_C_n[0], 9U * sizeof(real_T));
          }
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b((emlrtConstCTX)sp);
          }
        }
        covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 4, 0);
        covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 8U);
        /* 'compute_potential_energy_term:53' t1(j) = r_n1_n(:,n)' * dC_10 * g0;
         */
        b_r_n1_n = 0.0;
        C_n_tmp = r_n1_n[3 * n];
        b_C_n_tmp = r_n1_n[3 * n + 1];
        d = r_n1_n[3 * n + 2];
        for (i = 0; i < 3; i++) {
          b_r_n1_n += ((C_n_tmp * dC_10[3 * i] + b_C_n_tmp * dC_10[3 * i + 1]) +
                       d * dC_10[3 * i + 2]) *
                      g0[i];
        }
        t1[j] = b_r_n1_n;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b((emlrtConstCTX)sp);
        }
      }
      covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 3, 0);
      /* %% PE terms that go from 1:n */
      /* 'compute_potential_energy_term:56' for j = 1:n */
      C_n_tmp = c_n[3 * n];
      b_C_n_tmp = c_n[3 * n + 1];
      d = c_n[3 * n + 2];
      for (j = 0; j <= n; j++) {
        covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 5, 1);
        covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 9U);
        /* 'compute_potential_energy_term:57' dC_10 = eye(3); */
        memset(&dC_10[0], 0, 9U * sizeof(real_T));
        dC_10[0] = 1.0;
        dC_10[4] = 1.0;
        dC_10[8] = 1.0;
        /* 'compute_potential_energy_term:59' for k = 1:n */
        for (k = 0; k <= n; k++) {
          covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 6, 1);
          /* This multiplies the rotation matricies successively */
          /*  until the link j is reached in which case the rate */
          /*  is multiplied by the overall rotation */
          /* 'compute_potential_energy_term:63' if (k == j) */
          if (covrtLogIf(&emlrtCoverageInstance, 9U, 0U, 2, k == j)) {
            covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 10U);
            /* 'compute_potential_energy_term:64' dC_10 = C_n_rate(:,:,k) *
             * dC_10; */
            for (i = 0; i < 3; i++) {
              i1 = i + 9 * k;
              for (q = 0; q < 3; q++) {
                b_C_n[i + 3 * q] = (C_n_rate[i1] * dC_10[3 * q] +
                                    C_n_rate[i1 + 3] * dC_10[3 * q + 1]) +
                                   C_n_rate[i1 + 6] * dC_10[3 * q + 2];
              }
            }
            memcpy(&dC_10[0], &b_C_n[0], 9U * sizeof(real_T));
          } else {
            covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 11U);
            /* 'compute_potential_energy_term:65' else */
            /* 'compute_potential_energy_term:66' dC_10 = C_n(:,:,k) * dC_10; */
            for (i = 0; i < 3; i++) {
              i1 = i + 9 * k;
              for (q = 0; q < 3; q++) {
                b_C_n[i + 3 * q] =
                    (C_n[i1] * dC_10[3 * q] + C_n[i1 + 3] * dC_10[3 * q + 1]) +
                    C_n[i1 + 6] * dC_10[3 * q + 2];
              }
            }
            memcpy(&dC_10[0], &b_C_n[0], 9U * sizeof(real_T));
          }
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b((emlrtConstCTX)sp);
          }
        }
        covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 6, 0);
        covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 12U);
        /* dot product */
        /* %%% made change here c_n(n) -> c_n(:,n) */
        /* 'compute_potential_energy_term:71' t2 = c_n(:,n)' * dC_10 * g0; */
        /* 'compute_potential_energy_term:73' dVdtheta_i(j) = -m_i*t1(j)-t2; */
        b_r_n1_n = 0.0;
        for (i = 0; i < 3; i++) {
          b_r_n1_n += ((C_n_tmp * dC_10[3 * i] + b_C_n_tmp * dC_10[3 * i + 1]) +
                       d * dC_10[3 * i + 2]) *
                      g0[i];
        }
        dVdtheta_i[j] = -m_i * t1[j] - b_r_n1_n;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b((emlrtConstCTX)sp);
        }
      }
      covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 5, 0);
    }
    covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 13U);
    /* 'compute_potential_energy_term:76' Pot = Pot + dVdtheta_i; */
    r = _mm_loadu_pd(&Pot[0]);
    r1 = _mm_loadu_pd(&dVdtheta_i[0]);
    _mm_storeu_pd(&Pot[0], _mm_add_pd(r, r1));
    r = _mm_loadu_pd(&Pot[2]);
    r1 = _mm_loadu_pd(&dVdtheta_i[2]);
    _mm_storeu_pd(&Pot[2], _mm_add_pd(r, r1));
    r = _mm_loadu_pd(&Pot[4]);
    r1 = _mm_loadu_pd(&dVdtheta_i[4]);
    _mm_storeu_pd(&Pot[4], _mm_add_pd(r, r1));
    r = _mm_loadu_pd(&Pot[6]);
    r1 = _mm_loadu_pd(&dVdtheta_i[6]);
    _mm_storeu_pd(&Pot[6], _mm_add_pd(r, r1));
    Pot[8] += dVdtheta_i[8];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtConstCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 9U, 0U, 1, 0);
}

/* End of code generation (compute_potential_energy_term.c) */
