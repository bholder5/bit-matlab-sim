/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * bit_one_step.c
 *
 * Code generation for function 'bit_one_step'
 *
 */

/* Include files */
#include "bit_one_step.h"
#include "axis2rot.h"
#include "bit_one_step_mex_data.h"
#include "chol.h"
#include "eml_int_forloop_overflow_check.h"
#include "mass_mat_func.h"
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    19,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    24,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo c_emlrtRSI = {
    25,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    26,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo e_emlrtRSI = {
    27,                                        /* lineNo */
    "bit_one_step",                            /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo g_emlrtRSI = {
    63,                               /* lineNo */
    "function_handle/parenReference", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/eml/+coder/+internal/"
    "function_handle.m" /* pathName */
};

static emlrtRSInfo h_emlrtRSI =
    {
        17,                              /* lineNo */
        "compute_potential_energy_term", /* fcnName */
        "/home/brad/bit-matlab-sim/Plant_functions/"
        "compute_potential_energy_term.m" /* pathName */
};

static emlrtRSInfo j_emlrtRSI = {
    9,                                                     /* lineNo */
    "RW_terms",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/RW_terms.m" /* pathName */
};

static emlrtRSInfo q_emlrtRSI = {
    14, /* lineNo */
    "@(y_true,tau_applied,dw_piv)bit_propagator(y_true,c_n,z_n,m_n,r_n1_n,m_w_"
    "n,p_n,k_d,b_d,g0,unlock,hs_rw_max,tau_applied,w_piv,piv"
    "_flag,dw_piv,tau_max_piv,thet_pit_nom)",  /* fcnName */
    "/home/brad/bit-matlab-sim/bit_one_step.m" /* pathName */
};

static emlrtRSInfo r_emlrtRSI = {
    20,                                                          /* lineNo */
    "bit_propagator",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m" /* pathName */
};

static emlrtRSInfo s_emlrtRSI = {
    29,                                                          /* lineNo */
    "bit_propagator",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m" /* pathName */
};

static emlrtRSInfo t_emlrtRSI = {
    37,                                                          /* lineNo */
    "bit_propagator",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m" /* pathName */
};

static emlrtRSInfo u_emlrtRSI = {
    39,                                                          /* lineNo */
    "bit_propagator",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m" /* pathName */
};

static emlrtRSInfo v_emlrtRSI = {
    43,                                                          /* lineNo */
    "bit_propagator",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m" /* pathName */
};

static emlrtRSInfo w_emlrtRSI = {
    62,                                                          /* lineNo */
    "bit_propagator",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m" /* pathName */
};

static emlrtRSInfo x_emlrtRSI = {
    67,                                                          /* lineNo */
    "bit_propagator",                                            /* fcnName */
    "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m" /* pathName */
};

static emlrtRSInfo y_emlrtRSI = {
    34,     /* lineNo */
    "chol", /* fcnName */
    "/usr/local/MATLAB/R2021b/toolbox/eml/lib/matlab/matfun/chol.m" /* pathName
                                                                     */
};

static emlrtBCInfo c_emlrtBCI =
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

/* Function Declarations */
static void bit_one_step_anonFcn1(const emlrtStack *sp, const real_T unlock[9],
                                  real_T w_piv, boolean_T piv_flag,
                                  real_T tau_max_piv, real_T thet_pit_nom,
                                  const real_T y_true[21],
                                  const real_T tau_applied[9], real_T dw_piv,
                                  real_T varargout_1[21]);

/* Function Definitions */
/*
 * @(y_true, tau_applied, dw_piv)
 */
static void bit_one_step_anonFcn1(const emlrtStack *sp, const real_T unlock[9],
                                  real_T w_piv, boolean_T piv_flag,
                                  real_T tau_max_piv, real_T thet_pit_nom,
                                  const real_T y_true[21],
                                  const real_T tau_applied[9], real_T dw_piv,
                                  real_T varargout_1[21])
{
  static const real_T c_n[27] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -30.5,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.4,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static const real_T r_n1_n[27] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, -61.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.4, 0.0, 0.0,   0.0};
  static const real_T k_d[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0017453292519943296,
                                0.0, 0.0, 0.0};
  static const real_T g0[3] = {0.0, 0.0, -9.72};
  static const int16_T iv[9] = {0, 0, 10, 0, 0, 1, 350, 73, 150};
  static const int8_T z_n[27] = {0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1,
                                 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T C_n_rate[81];
  real_T b_C_n[81];
  real_T b_dVdtheta_i[27];
  real_T b_r_tmp[27];
  real_T r_tmp[27];
  real_T s7[27];
  real_T C_n[9];
  real_T Pot[9];
  real_T b_tau_applied[9];
  real_T dC_10[9];
  real_T dVdtheta_i[9];
  real_T dtheta[9];
  real_T t1[9];
  real_T b_z_n[3];
  real_T d;
  real_T d1;
  real_T int_err;
  real_T m_i;
  real_T tau_piv;
  real_T vec_idx_0_tmp;
  real_T vec_idx_2;
  int32_T b_z_n_tmp;
  int32_T c_z_n_tmp;
  int32_T i;
  int32_T j;
  int32_T n;
  int32_T q;
  int32_T z_n_tmp;
  boolean_T x[3];
  boolean_T exitg1;
  boolean_T y;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  covrtLogFcn(&emlrtCoverageInstance, 0U, 1U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 1U);
  /* 'bit_one_step:14' @(y_true, tau_applied, dw_piv) bit_propagator(y_true,
   * c_n, z_n, m_n, r_n1_n, m_w_n, p_n, ...  */
  /* 'bit_one_step:15'     k_d, b_d, g0, unlock, hs_rw_max, tau_applied, w_piv,
   * piv_flag, dw_piv, tau_max_piv, thet_pit_nom) */
  st.site = &q_emlrtRSI;
  memcpy(&b_tau_applied[0], &tau_applied[0], 9U * sizeof(real_T));
  covrtLogFcn(&emlrtCoverageInstance, 3U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 3U, 0U);
  /* split the state */
  /* 'bit_propagator:6' theta = X(10:18); */
  /* 'bit_propagator:7' dtheta = X(1:9); */
  memcpy(&dtheta[0], &y_true[0], 9U * sizeof(real_T));
  /* 'bit_propagator:8' hs = X(19:21); */
  /* extract RW torque. */
  /* 'bit_propagator:11' tau_rw = tau_applied(7); */
  /* 'bit_propagator:12' tau_applied(7) = 0; */
  b_tau_applied[6] = 0.0;
  /*  set pivot speed in dtheta... */
  /* 'bit_propagator:15' if piv_flag == true */
  if (covrtLogIf(&emlrtCoverageInstance, 3U, 0U, 0, piv_flag)) {
    covrtLogBasicBlock(&emlrtCoverageInstance, 3U, 1U);
    /* 'bit_propagator:16' dtheta(6) = w_piv; */
    dtheta[5] = w_piv;
  }
  covrtLogBasicBlock(&emlrtCoverageInstance, 3U, 2U);
  /*     %% */
  /* 'bit_propagator:20' Pot = compute_potential_energy_term(theta, c_n, z_n,
   * m_n, r_n1_n, g0); */
  b_st.site = &r_emlrtRSI;
  covrtLogFcn(&emlrtCoverageInstance, 4U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 0U);
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
  C_n[0] = -0.0;
  C_n[4] = -0.0;
  C_n[8] = -0.0;
  for (n = 0; n < 9; n++) {
    covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 1U);
    /* 'compute_potential_energy_term:17' C_n(:,:,n) = axis2rot(z_n(:,n),
     * theta(n)); */
    z_n_tmp = z_n[3 * n];
    b_z_n[0] = z_n_tmp;
    b_z_n_tmp = z_n[3 * n + 1];
    b_z_n[1] = b_z_n_tmp;
    c_z_n_tmp = z_n[3 * n + 2];
    b_z_n[2] = c_z_n_tmp;
    c_st.site = &h_emlrtRSI;
    axis2rot(&c_st, b_z_n, y_true[n + 9], *(real_T(*)[9]) & b_C_n[9 * n]);
    /* Partial of Rot_n with respect to theta_n */
    /* 'compute_potential_energy_term:19' C_n_rate(:,:,n) = -1 * xmat(z_n(:,n))
     * * C_n(:,:,n); */
    covrtLogFcn(&emlrtCoverageInstance, 2U, 0U);
    covrtLogBasicBlock(&emlrtCoverageInstance, 2U, 0U);
    /* 'xmat:2' mat = [        0, -vec(3),  vec(2); */
    /* 'xmat:3'             vec(3),         0, -vec(1); */
    /* 'xmat:4'            -vec(2),  vec(1),         0]; */
    C_n[3] = c_z_n_tmp;
    C_n[6] = -(real_T)b_z_n_tmp;
    C_n[1] = -(real_T)c_z_n_tmp;
    C_n[7] = z_n_tmp;
    C_n[2] = b_z_n_tmp;
    C_n[5] = -(real_T)z_n_tmp;
    for (c_z_n_tmp = 0; c_z_n_tmp < 3; c_z_n_tmp++) {
      i = (int32_T)C_n[c_z_n_tmp];
      b_z_n_tmp = (int32_T)C_n[c_z_n_tmp + 3];
      q = (int32_T)C_n[c_z_n_tmp + 6];
      for (z_n_tmp = 0; z_n_tmp < 3; z_n_tmp++) {
        j = 3 * z_n_tmp + 9 * n;
        C_n_rate[(c_z_n_tmp + 3 * z_n_tmp) + 9 * n] =
            ((real_T)i * b_C_n[j] + (real_T)b_z_n_tmp * b_C_n[j + 1]) +
            (real_T)q * b_C_n[j + 2];
      }
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&b_st);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 0, 0);
  /*  Compute potential energy term. First loop is cycling through each */
  /*  koints contribution. */
  /* 'compute_potential_energy_term:24' for n = 1:ndof */
  for (n = 0; n < 9; n++) {
    covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 1, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 2U);
    /* 'compute_potential_energy_term:25' m_i = 0; */
    m_i = 0.0;
    /* 'compute_potential_energy_term:26' dVdtheta_i = zeros(ndof,1); */
    memset(&dVdtheta_i[0], 0, 9U * sizeof(real_T));
    /*  mass of remaining link (ex. 7 + 8 + 9) is totla mass at OF joint */
    /* 'compute_potential_energy_term:29' for q = n:ndof */
    c_z_n_tmp = 8 - n;
    for (q = 0; q <= c_z_n_tmp; q++) {
      covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 2, 1);
      covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 3U);
      /* 'compute_potential_energy_term:30' m_i = m_i + m_n(q); */
      i = (n + q) + 1;
      if (i > 9) {
        emlrtDynamicBoundsCheckR2012b(i, 1, 9, &c_emlrtBCI, &b_st);
      }
      m_i += (real_T)iv[i - 1];
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(&b_st);
      }
    }
    covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 2, 0);
    /* 'compute_potential_energy_term:33' if (m_i ~= 0) */
    if (covrtLogIf(&emlrtCoverageInstance, 4U, 0U, 0, m_i != 0.0)) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 4U);
      /* 'compute_potential_energy_term:34' t1 = zeros(ndof,1); */
      memset(&t1[0], 0, 9U * sizeof(real_T));
      /* 'compute_potential_energy_term:35' t2 = t1; */
      /* Cycling through all joints up to joint n, the jint we are */
      /* currently calculating the contribution from */
      /*  */
      /* terms up to n-1 links.  */
      /* 'compute_potential_energy_term:41' for j = 1:n-1 */
      for (j = 0; j < n; j++) {
        covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 3, 1);
        covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 5U);
        /* 'compute_potential_energy_term:42' dC_10 = eye(3); */
        memset(&dC_10[0], 0, 9U * sizeof(real_T));
        dC_10[0] = 1.0;
        dC_10[4] = 1.0;
        dC_10[8] = 1.0;
        /* 'compute_potential_energy_term:44' for k = 1:n-1 */
        for (z_n_tmp = 0; z_n_tmp < n; z_n_tmp++) {
          covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 4, 1);
          /* This multiplies the rotation matricies successively */
          /*  until the link j where the rate is inserted instead */
          /* 'compute_potential_energy_term:47' if (k == j) */
          if (covrtLogIf(&emlrtCoverageInstance, 4U, 0U, 1, z_n_tmp == j)) {
            covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 6U);
            /* 'compute_potential_energy_term:48' dC_10 = C_n_rate(:,:,k) *
             * dC_10; */
            for (c_z_n_tmp = 0; c_z_n_tmp < 3; c_z_n_tmp++) {
              i = c_z_n_tmp + 9 * z_n_tmp;
              for (b_z_n_tmp = 0; b_z_n_tmp < 3; b_z_n_tmp++) {
                C_n[c_z_n_tmp + 3 * b_z_n_tmp] =
                    (C_n_rate[i] * dC_10[3 * b_z_n_tmp] +
                     C_n_rate[i + 3] * dC_10[3 * b_z_n_tmp + 1]) +
                    C_n_rate[i + 6] * dC_10[3 * b_z_n_tmp + 2];
              }
            }
            memcpy(&dC_10[0], &C_n[0], 9U * sizeof(real_T));
          } else {
            covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 7U);
            /* 'compute_potential_energy_term:49' else */
            /* 'compute_potential_energy_term:50' dC_10 = C_n(:,:,k) * dC_10; */
            for (c_z_n_tmp = 0; c_z_n_tmp < 3; c_z_n_tmp++) {
              i = c_z_n_tmp + 9 * z_n_tmp;
              for (b_z_n_tmp = 0; b_z_n_tmp < 3; b_z_n_tmp++) {
                C_n[c_z_n_tmp + 3 * b_z_n_tmp] =
                    (b_C_n[i] * dC_10[3 * b_z_n_tmp] +
                     b_C_n[i + 3] * dC_10[3 * b_z_n_tmp + 1]) +
                    b_C_n[i + 6] * dC_10[3 * b_z_n_tmp + 2];
              }
            }
            memcpy(&dC_10[0], &C_n[0], 9U * sizeof(real_T));
          }
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b(&b_st);
          }
        }
        covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 4, 0);
        covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 8U);
        /* 'compute_potential_energy_term:53' t1(j) = r_n1_n(:,n)' * dC_10 * g0;
         */
        int_err = 0.0;
        tau_piv = r_n1_n[3 * n];
        d = r_n1_n[3 * n + 1];
        d1 = r_n1_n[3 * n + 2];
        for (c_z_n_tmp = 0; c_z_n_tmp < 3; c_z_n_tmp++) {
          int_err +=
              ((tau_piv * dC_10[3 * c_z_n_tmp] + d * dC_10[3 * c_z_n_tmp + 1]) +
               d1 * dC_10[3 * c_z_n_tmp + 2]) *
              g0[c_z_n_tmp];
        }
        t1[j] = int_err;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(&b_st);
        }
      }
      covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 3, 0);
      /* %% PE terms that go from 1:n */
      /* 'compute_potential_energy_term:56' for j = 1:n */
      for (j = 0; j <= n; j++) {
        covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 5, 1);
        covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 9U);
        /* 'compute_potential_energy_term:57' dC_10 = eye(3); */
        memset(&dC_10[0], 0, 9U * sizeof(real_T));
        dC_10[0] = 1.0;
        dC_10[4] = 1.0;
        dC_10[8] = 1.0;
        /* 'compute_potential_energy_term:59' for k = 1:n */
        for (z_n_tmp = 0; z_n_tmp <= n; z_n_tmp++) {
          covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 6, 1);
          /* This multiplies the rotation matricies successively */
          /*  until the link j is reached in which case the rate */
          /*  is multiplied by the overall rotation */
          /* 'compute_potential_energy_term:63' if (k == j) */
          if (covrtLogIf(&emlrtCoverageInstance, 4U, 0U, 2, z_n_tmp == j)) {
            covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 10U);
            /* 'compute_potential_energy_term:64' dC_10 = C_n_rate(:,:,k) *
             * dC_10; */
            for (c_z_n_tmp = 0; c_z_n_tmp < 3; c_z_n_tmp++) {
              i = c_z_n_tmp + 9 * z_n_tmp;
              for (b_z_n_tmp = 0; b_z_n_tmp < 3; b_z_n_tmp++) {
                C_n[c_z_n_tmp + 3 * b_z_n_tmp] =
                    (C_n_rate[i] * dC_10[3 * b_z_n_tmp] +
                     C_n_rate[i + 3] * dC_10[3 * b_z_n_tmp + 1]) +
                    C_n_rate[i + 6] * dC_10[3 * b_z_n_tmp + 2];
              }
            }
            memcpy(&dC_10[0], &C_n[0], 9U * sizeof(real_T));
          } else {
            covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 11U);
            /* 'compute_potential_energy_term:65' else */
            /* 'compute_potential_energy_term:66' dC_10 = C_n(:,:,k) * dC_10; */
            for (c_z_n_tmp = 0; c_z_n_tmp < 3; c_z_n_tmp++) {
              i = c_z_n_tmp + 9 * z_n_tmp;
              for (b_z_n_tmp = 0; b_z_n_tmp < 3; b_z_n_tmp++) {
                C_n[c_z_n_tmp + 3 * b_z_n_tmp] =
                    (b_C_n[i] * dC_10[3 * b_z_n_tmp] +
                     b_C_n[i + 3] * dC_10[3 * b_z_n_tmp + 1]) +
                    b_C_n[i + 6] * dC_10[3 * b_z_n_tmp + 2];
              }
            }
            memcpy(&dC_10[0], &C_n[0], 9U * sizeof(real_T));
          }
          if (*emlrtBreakCheckR2012bFlagVar != 0) {
            emlrtBreakCheckR2012b(&b_st);
          }
        }
        covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 6, 0);
        covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 12U);
        /* dot product */
        /* %%% made change here c_n(n) -> c_n(:,n) */
        /* 'compute_potential_energy_term:71' t2 = c_n(:,n)' * dC_10 * g0; */
        /* 'compute_potential_energy_term:73' dVdtheta_i(j) = -m_i*t1(j)-t2; */
        int_err = 0.0;
        tau_piv = c_n[3 * n];
        d = c_n[3 * n + 1];
        d1 = c_n[3 * n + 2];
        for (c_z_n_tmp = 0; c_z_n_tmp < 3; c_z_n_tmp++) {
          int_err +=
              ((tau_piv * dC_10[3 * c_z_n_tmp] + d * dC_10[3 * c_z_n_tmp + 1]) +
               d1 * dC_10[3 * c_z_n_tmp + 2]) *
              g0[c_z_n_tmp];
        }
        dVdtheta_i[j] = -m_i * t1[j] - int_err;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(&b_st);
        }
      }
      covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 5, 0);
    }
    covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 13U);
    /* 'compute_potential_energy_term:76' Pot = Pot + dVdtheta_i; */
    for (c_z_n_tmp = 0; c_z_n_tmp < 9; c_z_n_tmp++) {
      Pot[c_z_n_tmp] += dVdtheta_i[c_z_n_tmp];
    }
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&b_st);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 4U, 0U, 1, 0);
  /* 'bit_propagator:22' theta_spring = theta; */
  memcpy(&t1[0], &y_true[9], 9U * sizeof(real_T));
  /* 'bit_propagator:23' theta_spring(9) = theta(9) - thet_pit_nom; */
  t1[8] = y_true[17] - thet_pit_nom;
  /* 'bit_propagator:25' spring = k_d.*theta_spring; */
  /* 'bit_propagator:26' damp = b_d.*dtheta; */
  /* place holder */
  /* 'bit_propagator:29' [R,r, d_hs] = RW_terms(theta, dtheta, z_n, hs, tau_rw,
   * hs_rw_max); */
  b_st.site = &s_emlrtRSI;
  covrtLogFcn(&emlrtCoverageInstance, 6U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 6U, 0U);
  /* UNTITLED Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* calculate the mapping matrix from dtheta to omega */
  /* 'RW_terms:7' s7 = zeros(3,9); */
  memset(&s7[0], 0, 27U * sizeof(real_T));
  /* 'RW_terms:8' for i = 1:7 */
  for (n = 0; n < 7; n++) {
    covrtLogFor(&emlrtCoverageInstance, 6U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 6U, 1U);
    /* 'RW_terms:9' Cn = axis2rot(z_n(:,i), theta(i)); */
    z_n_tmp = z_n[3 * n];
    b_z_n[0] = z_n_tmp;
    b_z_n_tmp = 3 * n + 1;
    c_z_n_tmp = z_n[b_z_n_tmp];
    b_z_n[1] = c_z_n_tmp;
    q = 3 * n + 2;
    j = z_n[q];
    b_z_n[2] = j;
    c_st.site = &j_emlrtRSI;
    axis2rot(&c_st, b_z_n, y_true[n + 9], C_n);
    memcpy(&dVdtheta_i[0], &C_n[0], 9U * sizeof(real_T));
    /* 'RW_terms:10' s7(:,i) = z_n(:,i); */
    s7[3 * n] = z_n_tmp;
    s7[b_z_n_tmp] = c_z_n_tmp;
    s7[q] = j;
    /* 'RW_terms:11' s7 = Cn*s7; */
    for (c_z_n_tmp = 0; c_z_n_tmp < 3; c_z_n_tmp++) {
      tau_piv = dVdtheta_i[c_z_n_tmp];
      d = dVdtheta_i[c_z_n_tmp + 3];
      d1 = dVdtheta_i[c_z_n_tmp + 6];
      for (i = 0; i < 9; i++) {
        b_dVdtheta_i[c_z_n_tmp + 3 * i] =
            (tau_piv * s7[3 * i] + d * s7[3 * i + 1]) + d1 * s7[3 * i + 2];
      }
    }
    memcpy(&s7[0], &b_dVdtheta_i[0], 27U * sizeof(real_T));
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&b_st);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 6U, 0U, 0, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 6U, 2U);
  /* 'RW_terms:15' d_hs = tau_rw*z_n(:,7); */
  /* 'RW_terms:17' if hs(3) >= hs_rw_max */
  vec_idx_0_tmp = tau_applied[6] * 0.0;
  x[0] = (y_true[20] >= 0.0);
  x[1] = (y_true[20] >= 0.0);
  vec_idx_2 = tau_applied[6];
  x[2] = (y_true[20] >= 28.274333882308138);
  y = true;
  z_n_tmp = 0;
  exitg1 = false;
  while ((!exitg1) && (z_n_tmp < 3)) {
    if (!x[z_n_tmp]) {
      y = false;
      exitg1 = true;
    } else {
      z_n_tmp++;
    }
  }
  if (covrtLogIf(&emlrtCoverageInstance, 6U, 0U, 0, y)) {
    /* 'RW_terms:18' if d_hs(3) > 0 */
    if (covrtLogIf(&emlrtCoverageInstance, 6U, 0U, 1, tau_applied[6] > 0.0)) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 6U, 3U);
      /* 'RW_terms:19' d_hs(3) = 0; */
      vec_idx_2 = 0.0;
    }
  } else {
    x[0] = (y_true[20] <= -0.0);
    x[1] = (y_true[20] <= -0.0);
    x[2] = (y_true[20] <= -28.274333882308138);
    y = true;
    z_n_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (z_n_tmp < 3)) {
      if (!x[z_n_tmp]) {
        y = false;
        exitg1 = true;
      } else {
        z_n_tmp++;
      }
    }
    if (covrtLogIf(&emlrtCoverageInstance, 6U, 0U, 2, y) &&
        covrtLogIf(&emlrtCoverageInstance, 6U, 0U, 3, tau_applied[6] < 0.0)) {
      /* 'RW_terms:21' elseif hs(3) <= -hs_rw_max */
      /* 'RW_terms:22' if d_hs(3) < 0 */
      covrtLogBasicBlock(&emlrtCoverageInstance, 6U, 4U);
      /* 'RW_terms:23' d_hs(3) = 0; */
      vec_idx_2 = 0.0;
    }
  }
  covrtLogBasicBlock(&emlrtCoverageInstance, 6U, 5U);
  /* 'RW_terms:27' r = (s7')*d_hs; */
  for (c_z_n_tmp = 0; c_z_n_tmp < 3; c_z_n_tmp++) {
    for (i = 0; i < 9; i++) {
      b_dVdtheta_i[i + 9 * c_z_n_tmp] = s7[c_z_n_tmp + 3 * i];
    }
  }
  /* 'RW_terms:29' R = -(s7') * xmat(hs) * s7 * dtheta; */
  covrtLogFcn(&emlrtCoverageInstance, 2U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 2U, 0U);
  /* 'xmat:2' mat = [        0, -vec(3),  vec(2); */
  /* 'xmat:3'             vec(3),         0, -vec(1); */
  /* 'xmat:4'            -vec(2),  vec(1),         0]; */
  /*  rw_g2 = 2.353962297635081; */
  /*  rw_g1 = 0.034; */
  /*  w_piv = rw_g1*((-hs(3)/i_rw(3,3))-w_rw_nom) - (rw_g2*tau_rw); */
  /* calculate joint torques from gravity elasticity and damnping according */
  /* to eq 3.37 */
  /* 'bit_propagator:33' torques = tau_applied - (Pot + spring + damp + R + r);
   */
  for (c_z_n_tmp = 0; c_z_n_tmp < 27; c_z_n_tmp++) {
    r_tmp[c_z_n_tmp] = -b_dVdtheta_i[c_z_n_tmp];
  }
  C_n[0] = 0.0;
  C_n[3] = -y_true[20];
  C_n[6] = y_true[19];
  C_n[1] = y_true[20];
  C_n[4] = 0.0;
  C_n[7] = -y_true[18];
  C_n[2] = -y_true[19];
  C_n[5] = y_true[18];
  C_n[8] = 0.0;
  for (c_z_n_tmp = 0; c_z_n_tmp < 9; c_z_n_tmp++) {
    tau_piv = r_tmp[c_z_n_tmp];
    d = r_tmp[c_z_n_tmp + 9];
    d1 = r_tmp[c_z_n_tmp + 18];
    for (i = 0; i < 3; i++) {
      b_r_tmp[c_z_n_tmp + 9 * i] =
          (tau_piv * C_n[3 * i] + d * C_n[3 * i + 1]) + d1 * C_n[3 * i + 2];
    }
    tau_piv = 0.0;
    d = b_r_tmp[c_z_n_tmp];
    d1 = b_r_tmp[c_z_n_tmp + 9];
    m_i = b_r_tmp[c_z_n_tmp + 18];
    for (i = 0; i < 9; i++) {
      tau_piv += ((d * s7[3 * i] + d1 * s7[3 * i + 1]) + m_i * s7[3 * i + 2]) *
                 dtheta[i];
    }
    tau_piv = b_tau_applied[c_z_n_tmp] -
              ((((Pot[c_z_n_tmp] + k_d[c_z_n_tmp] * t1[c_z_n_tmp]) +
                 0.0 * dtheta[c_z_n_tmp]) +
                tau_piv) +
               ((b_dVdtheta_i[c_z_n_tmp] * vec_idx_0_tmp +
                 b_dVdtheta_i[c_z_n_tmp + 9] * vec_idx_0_tmp) +
                b_dVdtheta_i[c_z_n_tmp + 18] * vec_idx_2));
    b_tau_applied[c_z_n_tmp] = tau_piv;
    dC_10[c_z_n_tmp] = tau_piv;
  }
  /*      M = compute_mass_matrix(theta, z_n, r_n1_n, m_w_n, p_n); */
  /* 'bit_propagator:37' M = mass_mat_func(theta); */
  b_st.site = &t_emlrtRSI;
  mass_mat_func(*(real_T(*)[9]) & y_true[9], C_n_rate);
  memcpy(&b_C_n[0], &C_n_rate[0], 81U * sizeof(real_T));
  /* 'bit_propagator:39' M_decomp = chol(M); */
  b_st.site = &u_emlrtRSI;
  c_st.site = &y_emlrtRSI;
  cholesky(&c_st, b_C_n);
  /* 'bit_propagator:43' ddtheta = M_decomp\((M_decomp')\torques); */
  for (c_z_n_tmp = 0; c_z_n_tmp < 9; c_z_n_tmp++) {
    dVdtheta_i[c_z_n_tmp] = dC_10[c_z_n_tmp];
    for (i = 0; i < 9; i++) {
      C_n_rate[i + 9 * c_z_n_tmp] = b_C_n[c_z_n_tmp + 9 * i];
    }
  }
  b_st.site = &v_emlrtRSI;
  mldivide(&b_st, C_n_rate, dVdtheta_i);
  b_st.site = &v_emlrtRSI;
  mldivide(&b_st, b_C_n, dVdtheta_i);
  /* 'bit_propagator:44' ddtheta = ddtheta.*unlock; */
  for (c_z_n_tmp = 0; c_z_n_tmp < 9; c_z_n_tmp++) {
    dVdtheta_i[c_z_n_tmp] *= unlock[c_z_n_tmp];
  }
  /* 'bit_propagator:46' if piv_flag == true */
  if (covrtLogIf(&emlrtCoverageInstance, 3U, 0U, 1, piv_flag)) {
    covrtLogBasicBlock(&emlrtCoverageInstance, 3U, 3U);
    /* 'bit_propagator:47' prop_err = 10; */
    /* 'bit_propagator:48' int_err = 0; */
    /* 'bit_propagator:49' kp = 1; */
    /* 'bit_propagator:50' ki = 0.5; */
    /* 'bit_propagator:51' prop_err = dw_piv - ddtheta(6); */
    m_i = dw_piv - dVdtheta_i[5];
    /* 'bit_propagator:52' int_err = int_err + prop_err; */
    int_err = m_i;
    /* 'bit_propagator:53' tau_piv = torques(6); */
    tau_piv = dC_10[5];
    /* 'bit_propagator:55' while abs(prop_err) > 1e-9 */
    exitg1 = false;
    while ((!exitg1) && covrtLogWhile(&emlrtCoverageInstance, 3U, 0U, 0,
                                      muDoubleScalarAbs(m_i) > 1.0E-9)) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 3U, 4U);
      /* 'bit_propagator:57' tau_piv = tau_piv + ((kp*prop_err) + (ki*int_err));
       */
      tau_piv += m_i + 0.5 * int_err;
      /* 'bit_propagator:58' if abs(tau_piv) > tau_max_piv */
      if (covrtLogIf(&emlrtCoverageInstance, 3U, 0U, 2,
                     muDoubleScalarAbs(tau_piv) > tau_max_piv)) {
        covrtLogBasicBlock(&emlrtCoverageInstance, 3U, 5U);
        /* 'bit_propagator:59' tau_piv = sign(tau_piv) * tau_max_piv; */
        dC_10[5] = muDoubleScalarSign(tau_piv) * tau_max_piv;
        /* 'bit_propagator:60' torques(6) = tau_piv; */
        /* 'bit_propagator:62' ddtheta = M_decomp\((M_decomp')\torques); */
        for (c_z_n_tmp = 0; c_z_n_tmp < 9; c_z_n_tmp++) {
          for (i = 0; i < 9; i++) {
            C_n_rate[i + 9 * c_z_n_tmp] = b_C_n[c_z_n_tmp + 9 * i];
          }
        }
        b_st.site = &w_emlrtRSI;
        mldivide(&b_st, C_n_rate, dC_10);
        memcpy(&dVdtheta_i[0], &dC_10[0], 9U * sizeof(real_T));
        b_st.site = &w_emlrtRSI;
        mldivide(&b_st, b_C_n, dVdtheta_i);
        exitg1 = true;
      } else {
        covrtLogBasicBlock(&emlrtCoverageInstance, 3U, 6U);
        /* 'bit_propagator:65' torques(6) = tau_piv; */
        dC_10[5] = tau_piv;
        /* 'bit_propagator:67' ddtheta = M_decomp\((M_decomp')\torques); */
        for (c_z_n_tmp = 0; c_z_n_tmp < 9; c_z_n_tmp++) {
          dVdtheta_i[c_z_n_tmp] = dC_10[c_z_n_tmp];
          for (i = 0; i < 9; i++) {
            C_n_rate[i + 9 * c_z_n_tmp] = b_C_n[c_z_n_tmp + 9 * i];
          }
        }
        b_st.site = &x_emlrtRSI;
        mldivide(&b_st, C_n_rate, dVdtheta_i);
        b_st.site = &x_emlrtRSI;
        mldivide(&b_st, b_C_n, dVdtheta_i);
        /* 'bit_propagator:68' prop_err = dw_piv - ddtheta(6); */
        m_i = dw_piv - dVdtheta_i[5];
        /* 'bit_propagator:69' int_err = int_err + prop_err; */
        int_err += m_i;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(&st);
        }
      }
    }
  }
  covrtLogBasicBlock(&emlrtCoverageInstance, 3U, 7U);
  /* 'bit_propagator:73' Xdot = [ddtheta; dtheta; d_hs;]; */
  for (n = 0; n < 9; n++) {
    varargout_1[n] = dVdtheta_i[n];
    varargout_1[n + 9] = dtheta[n];
  }
  varargout_1[18] = vec_idx_0_tmp;
  varargout_1[19] = vec_idx_0_tmp;
  varargout_1[20] = vec_idx_2;
}

/*
 * function [y_true] = bit_one_step(x0, tau_applied, unlock, w_piv, piv_flag,...
 *     dt, num_steps, tau_max_piv, thet_pit_nom)
 */
void bit_one_step(const emlrtStack *sp, const real_T x0[21],
                  const real_T tau_applied[9], const real_T unlock[9],
                  real_T w_piv, boolean_T piv_flag, real_T dt,
                  uint16_T num_steps, real_T tau_max_piv, real_T thet_pit_nom,
                  real_T y_true[21])
{
  emlrtStack b_st;
  emlrtStack st;
  real_T b_y_true[21];
  real_T k1[21];
  real_T k2[21];
  real_T k3[21];
  real_T varargout_1[21];
  real_T dw_piv;
  int32_T b_i;
  int32_T i;
  int32_T step;
  boolean_T th_over[9];
  boolean_T th_under[9];
  boolean_T overflow;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  covrtLogFcn(&emlrtCoverageInstance, 0U, 0U);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 0U);
  /*  Run initialization script */
  /* 'bit_one_step:4' [ndof, g0, r_n1_n, z_n, p_n, m_n, c_n, ... */
  /* 'bit_one_step:5'     i_n, m_w_n,  i_rw, bear_k_cst, bear_c_cst, k_d, b_d,
   * ... */
  /* 'bit_one_step:6'     w_rw_max, w_rw_nom, hs_rw, hs_rw_max] = init_func();
   */
  /*     %% Setup Simulation */
  /*  initial conditions, state is dtheta; theta */
  /* 'bit_one_step:9' y_true = x0; */
  memcpy(&y_true[0], &x0[0], 21U * sizeof(real_T));
  /*  Sim Parameters */
  /*  y_all1 = zeros(18, tf/(dt)); */
  /* 'bit_one_step:12' step = 0; */
  /* 'bit_one_step:14' sys = @(y_true, tau_applied, dw_piv)
   * bit_propagator(y_true, c_n, z_n, m_n, r_n1_n, m_w_n, p_n, ...  */
  /* 'bit_one_step:15'     k_d, b_d, g0, unlock, hs_rw_max, tau_applied, w_piv,
   * piv_flag, dw_piv, tau_max_piv, thet_pit_nom) */
  /*  sim */
  /* 'bit_one_step:19' for step = 1:num_steps */
  st.site = &emlrtRSI;
  overflow = ((1 <= num_steps) && (num_steps > 65534));
  if (overflow) {
    b_st.site = &f_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }
  i = num_steps;
  for (step = 0; step < i; step++) {
    covrtLogFor(&emlrtCoverageInstance, 0U, 0U, 0, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 0U, 2U);
    /*         %% Propagate the system */
    /* RK4 solver */
    /* 'bit_one_step:22' dw_piv = (w_piv - y_true(6))/dt; */
    dw_piv = (w_piv - y_true[5]) / dt;
    /* 'bit_one_step:24' k1 = sys(y_true, tau_applied, dw_piv) * dt; */
    st.site = &b_emlrtRSI;
    b_st.site = &g_emlrtRSI;
    bit_one_step_anonFcn1(&b_st, unlock, w_piv, piv_flag, tau_max_piv,
                          thet_pit_nom, y_true, tau_applied, dw_piv, k1);
    for (b_i = 0; b_i < 21; b_i++) {
      k1[b_i] *= dt;
    }
    /* 'bit_one_step:25' k2 = sys(y_true + (k1/2), tau_applied, dw_piv) * dt; */
    st.site = &c_emlrtRSI;
    for (b_i = 0; b_i < 21; b_i++) {
      b_y_true[b_i] = y_true[b_i] + k1[b_i] / 2.0;
    }
    b_st.site = &g_emlrtRSI;
    bit_one_step_anonFcn1(&b_st, unlock, w_piv, piv_flag, tau_max_piv,
                          thet_pit_nom, b_y_true, tau_applied, dw_piv, k2);
    for (b_i = 0; b_i < 21; b_i++) {
      k2[b_i] *= dt;
    }
    /* 'bit_one_step:26' k3 = sys(y_true + (k2/2), tau_applied, dw_piv) * dt; */
    st.site = &d_emlrtRSI;
    for (b_i = 0; b_i < 21; b_i++) {
      b_y_true[b_i] = y_true[b_i] + k2[b_i] / 2.0;
    }
    b_st.site = &g_emlrtRSI;
    bit_one_step_anonFcn1(&b_st, unlock, w_piv, piv_flag, tau_max_piv,
                          thet_pit_nom, b_y_true, tau_applied, dw_piv, k3);
    for (b_i = 0; b_i < 21; b_i++) {
      k3[b_i] *= dt;
    }
    /* 'bit_one_step:27' k4 = sys(y_true + k3, tau_applied, dw_piv) * dt; */
    st.site = &e_emlrtRSI;
    for (b_i = 0; b_i < 21; b_i++) {
      b_y_true[b_i] = y_true[b_i] + k3[b_i];
    }
    b_st.site = &g_emlrtRSI;
    bit_one_step_anonFcn1(&b_st, unlock, w_piv, piv_flag, tau_max_piv,
                          thet_pit_nom, b_y_true, tau_applied, dw_piv,
                          varargout_1);
    /* 'bit_one_step:29' tdd = ((k1+(2*k2)+(2*k3)+k4)/6); */
    /* 'bit_one_step:30' y_true = y_true + tdd; */
    for (b_i = 0; b_i < 21; b_i++) {
      y_true[b_i] += (((k1[b_i] + 2.0 * k2[b_i]) + 2.0 * k3[b_i]) +
                      varargout_1[b_i] * dt) /
                     6.0;
    }
    /* 'bit_one_step:32' th_over = y_true(10:18) > pi; */
    /* 'bit_one_step:33' th_under = y_true(10:18) < -pi; */
    for (b_i = 0; b_i < 9; b_i++) {
      dw_piv = y_true[b_i + 9];
      th_over[b_i] = (dw_piv > 3.1415926535897931);
      th_under[b_i] = (dw_piv < -3.1415926535897931);
    }
    /* 'bit_one_step:34' y_true(10:14) = y_true(10:14) -(2*pi*th_over(1:5)) +
     * (2*pi*th_under(1:5)); */
    for (b_i = 0; b_i < 5; b_i++) {
      y_true[b_i + 9] =
          (y_true[b_i + 9] - 6.2831853071795862 * (real_T)th_over[b_i]) +
          6.2831853071795862 * (real_T)th_under[b_i];
    }
    /* 'bit_one_step:35' y_true(16:18) = y_true(16:18) -(2*pi*th_over(7:9)) +
     * (2*pi*th_under(7:9)); */
    y_true[15] = (y_true[15] - 6.2831853071795862 * (real_T)th_over[6]) +
                 6.2831853071795862 * (real_T)th_under[6];
    y_true[16] = (y_true[16] - 6.2831853071795862 * (real_T)th_over[7]) +
                 6.2831853071795862 * (real_T)th_under[7];
    y_true[17] = (y_true[17] - 6.2831853071795862 * (real_T)th_over[8]) +
                 6.2831853071795862 * (real_T)th_under[8];
    /*          fprintf('current state:  %0.15f \n  %0.15f \n %0.15f \n %0.15f
     * \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n
     * %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n %0.15f \n
     * %0.15f \n %0.15f \n %0.15f \n \n', ... */
    /*              y_true(1), y_true(2), y_true(3), y_true(4), y_true(5),
     * y_true(6),... */
    /*               y_true(7), y_true(8), y_true(9), y_true(10), y_true(11),
     * y_true(12),... */
    /*                y_true(13), y_true(14), y_true(15), y_true(16),
     * y_true(17), y_true(18),... */
    /*                 y_true(19), y_true(20), y_true(21));         */
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b((emlrtCTX)sp);
    }
  }
  covrtLogFor(&emlrtCoverageInstance, 0U, 0U, 0, 0);
}

/* End of code generation (bit_one_step.c) */
