/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 *
 * bit_propagator.c
 *
 * Code generation for function 'bit_propagator'
 *
 */

/* Include files */
#include "bit_propagator.h"
#include "axis2rot.h"
#include "chol.h"
#include "main_func_data.h"
#include "mldivide.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo q_emlrtRSI = {
    16,                                                          /* lineNo */
    "bit_propagator",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m" /* pathName */
};

static emlrtRSInfo r_emlrtRSI = {
    22,                                                          /* lineNo */
    "bit_propagator",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m" /* pathName */
};

static emlrtRSInfo s_emlrtRSI = {
    31,                                                          /* lineNo */
    "bit_propagator",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m" /* pathName */
};

static emlrtRSInfo t_emlrtRSI = {
    34,                                                          /* lineNo */
    "bit_propagator",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m" /* pathName */
};

static emlrtRSInfo u_emlrtRSI = {
    35,                                                          /* lineNo */
    "bit_propagator",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m" /* pathName */
};

static emlrtRSInfo v_emlrtRSI =
    {
        17,                              /* lineNo */
        "compute_potential_energy_term", /* fcnName */
        "/home/brad/bit-matlab-sim/Plant_functions/"
        "compute_potential_energy_term.m" /* pathName */
};

static emlrtRSInfo x_emlrtRSI = {
    9,                                                     /* lineNo */
    "RW_terms",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/RW_terms.m" /* pathName */
};

static emlrtRSInfo ab_emlrtRSI = {
    20,                    /* lineNo */
    "compute_mass_matrix", /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/compute_mass_matrix.m" /* pathName
                                                                       */
};

static emlrtRSInfo cb_emlrtRSI = {
    12,     /* lineNo */
    "chol", /* fcnName */
    "/usr/local/MATLAB/R2022a/toolbox/eml/lib/matlab/matfun/chol.m" /* pathName
                                                                     */
};

static emlrtBCInfo c_emlrtBCI = {
    1,                     /* iFirst */
    9,                     /* iLast */
    37,                    /* lineNo */
    32,                    /* colNo */
    "T_n",                 /* aName */
    "compute_mass_matrix", /* fName */
    "/home/brad/bit-matlab-sim/Plant_functions/compute_mass_matrix.m", /* pName
                                                                        */
    0 /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    1,                     /* iFirst */
    9,                     /* iLast */
    45,                    /* lineNo */
    32,                    /* colNo */
    "T_n",                 /* aName */
    "compute_mass_matrix", /* fName */
    "/home/brad/bit-matlab-sim/Plant_functions/compute_mass_matrix.m", /* pName
                                                                        */
    0 /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = {
    1,                     /* iFirst */
    9,                     /* iLast */
    52,                    /* lineNo */
    35,                    /* colNo */
    "p_n",                 /* aName */
    "compute_mass_matrix", /* fName */
    "/home/brad/bit-matlab-sim/Plant_functions/compute_mass_matrix.m", /* pName
                                                                        */
    0 /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    1,                     /* iFirst */
    9,                     /* iLast */
    54,                    /* lineNo */
    38,                    /* colNo */
    "m_w_n",               /* aName */
    "compute_mass_matrix", /* fName */
    "/home/brad/bit-matlab-sim/Plant_functions/compute_mass_matrix.m", /* pName
                                                                        */
    0 /* checkKind */
};

static emlrtBCInfo g_emlrtBCI =
    {
        1,                               /* iFirst */
        9,                               /* iLast */
        30,                              /* lineNo */
        29,                              /* colNo */
        "m_n",                           /* aName */
        "compute_potential_energy_term", /* fName */
        "/home/brad/bit-matlab-sim/Plant_functions/"
        "compute_potential_energy_term.m", /* pName */
        0                                  /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = {
    1,                     /* iFirst */
    9,                     /* iLast */
    57,                    /* lineNo */
    20,                    /* colNo */
    "mass_mat",            /* aName */
    "compute_mass_matrix", /* fName */
    "/home/brad/bit-matlab-sim/Plant_functions/compute_mass_matrix.m", /* pName
                                                                        */
    3 /* checkKind */
};

/* Function Definitions */
void bit_propagator(const emlrtStack *sp, const real_T X[21],
                    const real_T m_w_n[324], real_T tau_applied[9],
                    real_T Xdot[21])
{
  static const real_T b_r_n1_n[27] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, -61.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.4, 0.0, 0.0,   0.0};
  static const real_T b_dv[9] = {0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0017453292519943296,
                                 0.0,
                                 12.281626381649737,
                                 12.281626381649737};
  static const real_T b[3] = {0.0, 0.0, -9.72};
  static const int8_T p_n[54] = {0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                                 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1,
                                 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0};
  emlrtStack b_st;
  emlrtStack st;
  real_T T_n[324];
  real_T C_n[81];
  real_T C_n_rate[81];
  real_T T_nj[36];
  real_T b_T_n[36];
  real_T b_T_ni[36];
  real_T b_dVdtheta_i[27];
  real_T b_r_tmp[27];
  real_T r_tmp[27];
  real_T s7[27];
  real_T Pot[9];
  real_T b_C_n[9];
  real_T dC_10[9];
  real_T dVdtheta_i[9];
  real_T dtheta[9];
  real_T t1[9];
  real_T vec[3];
  real_T d;
  real_T d1;
  real_T d2;
  real_T d_hs_idx_0_tmp;
  real_T d_hs_idx_2;
  real_T m_i;
  real_T r_n1_n;
  real_T tau_rw;
  int32_T b_i;
  int32_T b_vec_tmp;
  int32_T c_vec_tmp;
  int32_T d_vec_tmp;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
  int32_T i4;
  int32_T j;
  int32_T k;
  int32_T n;
  int32_T q;
  int32_T vec_tmp;
  boolean_T x[3];
  boolean_T exitg1;
  boolean_T y;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  covrtLogFcn(&emlrtCoverageInstance, 9U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 9U, 0U);
  /* split the state */
  memcpy(&dtheta[0], &X[0], 9U * sizeof(real_T));
  /* extract RW torque. */
  tau_rw = tau_applied[6];
  tau_applied[6] = 0.0;
  /*      tau_applied = zeros(9,1); */
  /*     tau_rw = tau_applied(7); */
  /*     %% */
  st.site = &q_emlrtRSI;
  covrtLogFcn(&emlrtCoverageInstance, 10U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 0U);
  /*  This function is implementing the potential energy term of the */
  /*  lagrangian. See section 3.1.1 of Romualdez Thesis, specifically */
  /*  Eq3.25-3.26. The rate is because of the partial derivitive of V wrt */
  /*  theta.  */
  /* Determine relative rotation matrices and rates for each joint */
  /* rotation matricies */
  /*  Potential energy */
  memset(&Pot[0], 0, 9U * sizeof(real_T));
  b_C_n[0] = -0.0;
  b_C_n[4] = -0.0;
  b_C_n[8] = -0.0;
  for (n = 0; n < 9; n++) {
    covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 1U);
    vec_tmp = iv1[3 * n];
    vec[0] = vec_tmp;
    b_vec_tmp = iv1[3 * n + 1];
    vec[1] = b_vec_tmp;
    c_vec_tmp = iv1[3 * n + 2];
    vec[2] = c_vec_tmp;
    b_st.site = &v_emlrtRSI;
    axis2rot(&b_st, vec, X[n + 9], *(real_T(*)[9]) & C_n[9 * n]);
    /* Partial of Rot_n with respect to theta_n */
    covrtLogFcn(&emlrtCoverageInstance, 2U, 0U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 2U, 0U);
    b_C_n[3] = c_vec_tmp;
    b_C_n[6] = -(real_T)b_vec_tmp;
    b_C_n[1] = -(real_T)c_vec_tmp;
    b_C_n[7] = vec_tmp;
    b_C_n[2] = b_vec_tmp;
    b_C_n[5] = -(real_T)vec_tmp;
    for (i = 0; i < 3; i++) {
      i1 = (int32_T)b_C_n[i];
      q = (int32_T)b_C_n[i + 3];
      i2 = (int32_T)b_C_n[i + 6];
      for (i3 = 0; i3 < 3; i3++) {
        c_vec_tmp = 3 * i3 + 9 * n;
        C_n_rate[(i + 3 * i3) + 9 * n] =
            ((real_T)i1 * C_n[c_vec_tmp] + (real_T)q * C_n[c_vec_tmp + 1]) +
            (real_T)i2 * C_n[c_vec_tmp + 2];
      }
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&st);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 0, 0);
  /*  Compute potential energy term. First loop is cycling through each */
  /*  koints contribution. */
  for (n = 0; n < 9; n++) {
    covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 1, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 2U);
    m_i = 0.0;
    memset(&dVdtheta_i[0], 0, 9U * sizeof(real_T));
    /*  mass of remaining link (ex. 7 + 8 + 9) is totla mass at OF joint */
    i = 8 - n;
    for (q = 0; q <= i; q++) {
      covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 2, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 3U);
      i1 = (n + q) + 1;
      if (i1 > 9) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, 9, &g_emlrtBCI, &st);
      }
      m_i += (real_T)iv[i1 - 1];
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(&st);
      }
    }
    covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 2, 0);
    if (covrtLogIf(&emlrtCoverageInstance, 10U, 0U, 0, m_i != 0.0)) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 4U);
      memset(&t1[0], 0, 9U * sizeof(real_T));
      /* Cycling through all joints up to joint n, the jint we are */
      /* currently calculating the contribution from */
      /*  */
      /* terms up to n-1 links.  */
      for (j = 0; j < n; j++) {
        covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 3, 1);
        covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 5U);
        memset(&dC_10[0], 0, 9U * sizeof(real_T));
        dC_10[0] = 1.0;
        dC_10[4] = 1.0;
        dC_10[8] = 1.0;
        for (k = 0; k < n; k++) {
          covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 4, 1);
          /* This multiplies the rotation matricies successively */
          /*  until the link j where the rate is inserted instead */
          if (covrtLogIf(&emlrtCoverageInstance, 10U, 0U, 1, k == j)) {
            covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 6U);
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
            covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 7U);
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
            emlrtBreakCheckR2012b(&st);
          }
        }
        covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 4, 0);
        covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 8U);
        r_n1_n = 0.0;
        d = b_r_n1_n[3 * n];
        d1 = b_r_n1_n[3 * n + 1];
        d2 = b_r_n1_n[3 * n + 2];
        for (i = 0; i < 3; i++) {
          r_n1_n += ((d * dC_10[3 * i] + d1 * dC_10[3 * i + 1]) +
                     d2 * dC_10[3 * i + 2]) *
                    b[i];
        }
        t1[j] = r_n1_n;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(&st);
        }
      }
      covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 3, 0);
      /* %% PE terms that go from 1:n */
      d = dv[3 * n];
      d1 = dv[3 * n + 1];
      d2 = dv[3 * n + 2];
      for (j = 0; j <= n; j++) {
        covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 5, 1);
        covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 9U);
        memset(&dC_10[0], 0, 9U * sizeof(real_T));
        dC_10[0] = 1.0;
        dC_10[4] = 1.0;
        dC_10[8] = 1.0;
        for (k = 0; k <= n; k++) {
          covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 6, 1);
          /* This multiplies the rotation matricies successively */
          /*  until the link j is reached in which case the rate */
          /*  is multiplied by the overall rotation */
          if (covrtLogIf(&emlrtCoverageInstance, 10U, 0U, 2, k == j)) {
            covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 10U);
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
            covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 11U);
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
            emlrtBreakCheckR2012b(&st);
          }
        }
        covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 6, 0);
        covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 12U);
        /* dot product */
        /* %%% made change here c_n(n) -> c_n(:,n) */
        r_n1_n = 0.0;
        for (i = 0; i < 3; i++) {
          r_n1_n += ((d * dC_10[3 * i] + d1 * dC_10[3 * i + 1]) +
                     d2 * dC_10[3 * i + 2]) *
                    b[i];
        }
        dVdtheta_i[j] = -m_i * t1[j] - r_n1_n;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(&st);
        }
      }
      covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 5, 0);
    }
    covrtLogBasicBlock(&emlrtCoverageInstance, 10U, 13U);
    for (i = 0; i < 9; i++) {
      Pot[i] += dVdtheta_i[i];
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&st);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 10U, 0U, 1, 0);
  /* place holder */
  st.site = &r_emlrtRSI;
  covrtLogFcn(&emlrtCoverageInstance, 11U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 11U, 0U);
  /* UNTITLED Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* calculate the mapping matrix from dtheta to omega */
  memset(&s7[0], 0, 27U * sizeof(real_T));
  for (b_i = 0; b_i < 7; b_i++) {
    covrtLogFor(&emlrtCoverageInstance, 11U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 11U, 1U);
    vec_tmp = iv1[3 * b_i];
    vec[0] = vec_tmp;
    b_vec_tmp = 3 * b_i + 1;
    c_vec_tmp = iv1[b_vec_tmp];
    vec[1] = c_vec_tmp;
    q = 3 * b_i + 2;
    d_vec_tmp = iv1[q];
    vec[2] = d_vec_tmp;
    b_st.site = &x_emlrtRSI;
    axis2rot(&b_st, vec, X[b_i + 9], dVdtheta_i);
    s7[3 * b_i] = vec_tmp;
    s7[b_vec_tmp] = c_vec_tmp;
    s7[q] = d_vec_tmp;
    for (i = 0; i < 3; i++) {
      d = dVdtheta_i[i];
      d1 = dVdtheta_i[i + 3];
      d2 = dVdtheta_i[i + 6];
      for (i1 = 0; i1 < 9; i1++) {
        b_dVdtheta_i[i + 3 * i1] =
            (d * s7[3 * i1] + d1 * s7[3 * i1 + 1]) + d2 * s7[3 * i1 + 2];
      }
    }
    memcpy(&s7[0], &b_dVdtheta_i[0], 27U * sizeof(real_T));
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&st);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 11U, 0U, 0, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 11U, 2U);
  d_hs_idx_0_tmp = tau_rw * 0.0;
  x[0] = (X[20] >= 0.0);
  x[1] = (X[20] >= 0.0);
  d_hs_idx_2 = tau_rw;
  x[2] = (X[20] >= 56.548667764616276);
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (covrtLogIf(&emlrtCoverageInstance, 11U, 0U, 0, y)) {
    if (covrtLogIf(&emlrtCoverageInstance, 11U, 0U, 1, tau_rw > 0.0)) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 11U, 3U);
      d_hs_idx_2 = 0.0;
    }
  } else {
    x[0] = (X[20] <= -0.0);
    x[1] = (X[20] <= -0.0);
    x[2] = (X[20] <= -56.548667764616276);
    y = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 3)) {
      if (!x[k]) {
        y = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
    if (covrtLogIf(&emlrtCoverageInstance, 11U, 0U, 2, y) &&
        covrtLogIf(&emlrtCoverageInstance, 11U, 0U, 3, tau_rw < 0.0)) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 11U, 4U);
      d_hs_idx_2 = 0.0;
    }
  }
  covrtLogBasicBlock(&emlrtCoverageInstance, 11U, 5U);
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < 9; i1++) {
      b_dVdtheta_i[i1 + 9 * i] = s7[i + 3 * i1];
    }
  }
  covrtLogFcn(&emlrtCoverageInstance, 2U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 2U, 0U);
  dtheta[5] =
      -0.034 * (X[20] / 9.0 - 3.1415926535897931) - 2.3539622976350807 * tau_rw;
  /*  calculate the ddtheta resulting from desired  */
  /* calculate joint torques from gravity elasticity and damnping according */
  /* to eq 3.37 */
  st.site = &s_emlrtRSI;
  covrtLogFcn(&emlrtCoverageInstance, 12U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 12U, 0U);
  /* Compute_Mass_Matrix computes the mass matrix of the 9 state system using */
  /* the angles theta to calculate the interbody transformation matrices and */
  /* the axis of rotations. */
  /*  Initialize mass matrix */
  memset(&C_n[0], 0, 81U * sizeof(real_T));
  /*  memory to store the interbody transformations */
  /*  equation 3.8: Interbody transformations for each frame */
  b_C_n[0] = 0.0;
  b_C_n[4] = 0.0;
  b_C_n[8] = 0.0;
  for (b_i = 0; b_i < 9; b_i++) {
    covrtLogFor(&emlrtCoverageInstance, 12U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 12U, 1U);
    vec[0] = iv1[3 * b_i];
    vec_tmp = 3 * b_i + 1;
    vec[1] = iv1[vec_tmp];
    b_vec_tmp = 3 * b_i + 2;
    vec[2] = iv1[b_vec_tmp];
    b_st.site = &ab_emlrtRSI;
    axis2rot(&b_st, vec, X[b_i + 9], dVdtheta_i);
    vec[0] = b_r_n1_n[3 * b_i];
    vec[1] = b_r_n1_n[vec_tmp];
    vec[2] = b_r_n1_n[b_vec_tmp];
    covrtLogFcn(&emlrtCoverageInstance, 2U, 0U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 2U, 0U);
    b_C_n[3] = -vec[2];
    b_C_n[6] = vec[1];
    b_C_n[1] = vec[2];
    b_C_n[7] = -vec[0];
    b_C_n[2] = -vec[1];
    b_C_n[5] = vec[0];
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < 3; i1++) {
        dC_10[i + 3 * i1] = (-dVdtheta_i[i] * b_C_n[3 * i1] +
                             -dVdtheta_i[i + 3] * b_C_n[3 * i1 + 1]) +
                            -dVdtheta_i[i + 6] * b_C_n[3 * i1 + 2];
        T_n[(i1 + 6 * i) + 36 * b_i] = dVdtheta_i[i1 + 3 * i];
      }
    }
    for (i = 0; i < 3; i++) {
      q = 6 * (i + 3) + 36 * b_i;
      T_n[q] = dC_10[3 * i];
      d_vec_tmp = 6 * i + 36 * b_i;
      T_n[d_vec_tmp + 3] = 0.0;
      T_n[q + 3] = dVdtheta_i[3 * i];
      vec_tmp = 3 * i + 1;
      T_n[q + 1] = dC_10[vec_tmp];
      T_n[d_vec_tmp + 4] = 0.0;
      T_n[q + 4] = dVdtheta_i[vec_tmp];
      vec_tmp = 3 * i + 2;
      T_n[q + 2] = dC_10[vec_tmp];
      T_n[d_vec_tmp + 5] = 0.0;
      T_n[q + 5] = dVdtheta_i[vec_tmp];
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&st);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 12U, 0U, 0, 0);
  /*  Generate the mass matrix */
  /*  Eq 3.12-3.13 */
  for (b_i = 0; b_i < 9; b_i++) {
    covrtLogFor(&emlrtCoverageInstance, 12U, 0U, 1, 1);
    i = 8 - b_i;
    for (j = 0; j <= i; j++) {
      vec_tmp = (b_i + j) + 1;
      covrtLogFor(&emlrtCoverageInstance, 12U, 0U, 2, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 12U, 2U);
      m_i = 0.0;
      i1 = 9 - vec_tmp;
      if (i1 >= 0) {
        i4 = vec_tmp - b_i;
      }
      for (n = 0; n <= i1; n++) {
        real_T T_ni[6];
        b_vec_tmp = (vec_tmp + n) - 1;
        covrtLogFor(&emlrtCoverageInstance, 12U, 0U, 3, 1);
        covrtLogBasicBlock(&emlrtCoverageInstance, 12U, 3U);
        memset(&b_T_ni[0], 0, 36U * sizeof(real_T));
        for (k = 0; k < 6; k++) {
          b_T_ni[k + 6 * k] = 1.0;
        }
        memcpy(&T_nj[0], &b_T_ni[0], 36U * sizeof(real_T));
        for (k = 0; k <= i4 - 2; k++) {
          covrtLogFor(&emlrtCoverageInstance, 12U, 0U, 4, 1);
          covrtLogBasicBlock(&emlrtCoverageInstance, 12U, 4U);
          /*                  tic */
          d_vec_tmp = b_i + k;
          if (d_vec_tmp + 2 > 9) {
            emlrtDynamicBoundsCheckR2012b(d_vec_tmp + 2, 1, 9, &c_emlrtBCI,
                                          &st);
          }
          for (q = 0; q < 6; q++) {
            for (i2 = 0; i2 < 6; i2++) {
              d = 0.0;
              for (i3 = 0; i3 < 6; i3++) {
                d += T_n[(q + 6 * i3) + 36 * (d_vec_tmp + 1)] *
                     b_T_ni[i3 + 6 * i2];
              }
              b_T_n[q + 6 * i2] = d;
            }
          }
          memcpy(&b_T_ni[0], &b_T_n[0], 36U * sizeof(real_T));
          /*                  toc */
          /*                  tic */
          /*                  T_ni = mtimes(T_n(:,:,k),T_ni); */
          /*                  toc */
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b(&st);
          }
        }
        covrtLogFor(&emlrtCoverageInstance, 12U, 0U, 4, 0);
        q = b_vec_tmp - vec_tmp;
        for (k = 0; k <= q; k++) {
          covrtLogFor(&emlrtCoverageInstance, 12U, 0U, 5, 1);
          covrtLogBasicBlock(&emlrtCoverageInstance, 12U, 5U);
          d_vec_tmp = vec_tmp + k;
          if (d_vec_tmp + 1 > 9) {
            emlrtDynamicBoundsCheckR2012b(d_vec_tmp + 1, 1, 9, &d_emlrtBCI,
                                          &st);
          }
          for (i2 = 0; i2 < 6; i2++) {
            for (i3 = 0; i3 < 6; i3++) {
              d = 0.0;
              for (c_vec_tmp = 0; c_vec_tmp < 6; c_vec_tmp++) {
                d += T_n[(i2 + 6 * c_vec_tmp) + 36 * d_vec_tmp] *
                     T_nj[c_vec_tmp + 6 * i3];
              }
              b_T_n[i2 + 6 * i3] = d;
            }
          }
          memcpy(&T_nj[0], &b_T_n[0], 36U * sizeof(real_T));
          /*                  T_nj = mtimes(T_n(:,:,k),T_nj); */
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b(&st);
          }
        }
        covrtLogFor(&emlrtCoverageInstance, 12U, 0U, 5, 0);
        covrtLogBasicBlock(&emlrtCoverageInstance, 12U, 6U);
        for (q = 0; q < 6; q++) {
          for (i2 = 0; i2 < 6; i2++) {
            d = 0.0;
            for (i3 = 0; i3 < 6; i3++) {
              d += T_nj[q + 6 * i3] * b_T_ni[i3 + 6 * i2];
            }
            b_T_n[q + 6 * i2] = d;
          }
        }
        memcpy(&b_T_ni[0], &b_T_n[0], 36U * sizeof(real_T));
        /*              T_ni = mtimes(T_nj,T_ni); */
        if (vec_tmp > 9) {
          emlrtDynamicBoundsCheckR2012b(vec_tmp, 1, 9, &e_emlrtBCI, &st);
        }
        if (b_vec_tmp + 1 > 9) {
          emlrtDynamicBoundsCheckR2012b(b_vec_tmp + 1, 1, 9, &f_emlrtBCI, &st);
        }
        for (q = 0; q < 6; q++) {
          d = 0.0;
          for (i2 = 0; i2 < 6; i2++) {
            d += b_T_ni[q + 6 * i2] * (real_T)p_n[i2 + 6 * b_i];
          }
          T_ni[q] = d;
        }
        r_n1_n = 0.0;
        for (q = 0; q < 6; q++) {
          d = 0.0;
          d1 = 0.0;
          for (i2 = 0; i2 < 6; i2++) {
            d += T_ni[i2] * m_w_n[(i2 + 6 * q) + 36 * b_vec_tmp];
            d1 += T_nj[q + 6 * i2] * (real_T)p_n[i2 + 6 * (vec_tmp - 1)];
          }
          r_n1_n += d * d1;
        }
        m_i += r_n1_n;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(&st);
        }
      }
      covrtLogFor(&emlrtCoverageInstance, 12U, 0U, 3, 0);
      covrtLogBasicBlock(&emlrtCoverageInstance, 12U, 7U);
      if (vec_tmp > 9) {
        emlrtDynamicBoundsCheckR2012b(vec_tmp, 1, 9, &h_emlrtBCI, &st);
      }
      C_n[b_i + 9 * (vec_tmp - 1)] = m_i;
      if (covrtLogIf(&emlrtCoverageInstance, 12U, 0U, 0, b_i + 1 != vec_tmp)) {
        covrtLogBasicBlock(&emlrtCoverageInstance, 12U, 8U);
        C_n[(vec_tmp + 9 * b_i) - 1] = m_i;
      }
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(&st);
      }
    }
    covrtLogFor(&emlrtCoverageInstance, 12U, 0U, 2, 0);
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&st);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 12U, 0U, 1, 0);
  /*      Mass = mass_mat_func(theta); */
  st.site = &t_emlrtRSI;
  b_st.site = &cb_emlrtRSI;
  chol(&b_st, C_n);
  for (i = 0; i < 27; i++) {
    r_tmp[i] = -b_dVdtheta_i[i];
  }
  b_C_n[0] = 0.0;
  b_C_n[3] = -X[20];
  b_C_n[6] = X[19];
  b_C_n[1] = X[20];
  b_C_n[4] = 0.0;
  b_C_n[7] = -X[18];
  b_C_n[2] = -X[19];
  b_C_n[5] = X[18];
  b_C_n[8] = 0.0;
  for (i = 0; i < 9; i++) {
    d = r_tmp[i];
    d1 = r_tmp[i + 9];
    d2 = r_tmp[i + 18];
    for (i1 = 0; i1 < 3; i1++) {
      b_r_tmp[i + 9 * i1] =
          (d * b_C_n[3 * i1] + d1 * b_C_n[3 * i1 + 1]) + d2 * b_C_n[3 * i1 + 2];
    }
    d = 0.0;
    d1 = (b_dVdtheta_i[i] * d_hs_idx_0_tmp +
          b_dVdtheta_i[i + 9] * d_hs_idx_0_tmp) +
         b_dVdtheta_i[i + 18] * d_hs_idx_2;
    d2 = b_r_tmp[i];
    m_i = b_r_tmp[i + 9];
    r_n1_n = b_r_tmp[i + 18];
    for (i1 = 0; i1 < 9; i1++) {
      d +=
          ((d2 * s7[3 * i1] + m_i * s7[3 * i1 + 1]) + r_n1_n * s7[3 * i1 + 2]) *
          X[i1];
      C_n_rate[i1 + 9 * i] = C_n[i + 9 * i1];
    }
    d += (Pot[i] + b_dv[i] * X[i + 9]) + 0.0 * X[i];
    Pot[i] = d1;
    d = tau_applied[i] - (d + d1);
    dVdtheta_i[i] = d;
  }
  st.site = &u_emlrtRSI;
  mldivide(&st, C_n_rate, dVdtheta_i);
  st.site = &u_emlrtRSI;
  mldivide(&st, C_n, dVdtheta_i);
  /*  set pivot speed in dtheta... */
  for (b_i = 0; b_i < 9; b_i++) {
    Xdot[b_i] = dVdtheta_i[b_i];
    Xdot[b_i + 9] = dtheta[b_i];
  }
  Xdot[18] = d_hs_idx_0_tmp;
  Xdot[19] = d_hs_idx_0_tmp;
  Xdot[20] = d_hs_idx_2;
}

/* End of code generation (bit_propagator.c) */
