/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 * File: bit_one_step.c
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 18-Jan-2022 06:50:46
 */

/* Include Files */
#include "bit_one_step.h"
#include "bit_one_step_emxutil.h"
#include "bit_one_step_types.h"
#include "rt_nonfinite.h"
#include "omp.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Variable Definitions */
omp_nest_lock_t bit_one_step_nestLockGlobal;

static boolean_T isInitialized_bit_one_step = false;

/* Function Declarations */
static void axis2rot(const double v[3], double phi, double rot[9]);

static void bit_propagator(const double X[21], double tau_applied[9],
                           double Xdot[21]);

static int div_nde_s32_floor(int numerator);

static void mldivide(const double A[81], double B[9]);

/* Function Definitions */
/*
 * function rot = axis2rot( v, phi)
 *
 * This function gives the rotation matric applied to other rotation
 *  matricies, not the vector (it is transpose of the rot mat applied to the
 *  vector.
 *
 * Arguments    : const double v[3]
 *                double phi
 *                double rot[9]
 * Return Type  : void
 */
static void axis2rot(const double v[3], double phi, double rot[9])
{
  double b_sign;
  double cosa;
  double sina;
  int j;
  int k;
  /* 'axis2rot:5' cosa = cos(phi); */
  cosa = cos(phi);
  /* 'axis2rot:6' sina = sin(phi); */
  sina = sin(phi);
  /* 'axis2rot:8' sign = 1; */
  b_sign = 1.0;
  /* 'axis2rot:9' rot = (zeros(3,3)); */
  memset(&rot[0], 0, 9U * sizeof(double));
  /* 'axis2rot:11' for k = 1:3 */
  for (k = 0; k < 3; k++) {
    int i;
    /* 'axis2rot:12' for j = k:3 */
    i = 2 - k;
    for (j = 0; j <= i; j++) {
      double mij;
      int b_j;
      b_j = k + j;
      /* 'axis2rot:13' mij = (1-cosa)*v(k)*v(j); */
      mij = (1.0 - cosa) * v[k] * v[b_j];
      /* 'axis2rot:14' if (k == j) */
      if (k == b_j) {
        /* 'axis2rot:15' mij = mij + cosa; */
        rot[k + 3 * b_j] = mij + cosa;
        /* 'axis2rot:16' rot(k,j) = mij; */
      } else {
        double rot_tmp;
        /* 'axis2rot:17' else */
        /* index is 3 - j - k for 0 indexed programming languages */
        /* 'axis2rot:19' rot(k,j) = mij + (sign*sina*v((5-k-j)+1)); */
        rot_tmp = b_sign * sina * v[3 - (k + b_j)];
        rot[k + 3 * b_j] = mij + rot_tmp;
        /* 'axis2rot:20' rot(j,k) = mij - (sign*sina*v((5-k-j)+1)); */
        rot[b_j + 3 * k] = mij - rot_tmp;
        /* 'axis2rot:21' sign = sign*-1; */
        b_sign = -b_sign;
      }
    }
  }
}

/*
 * function Xdot = bit_propagator(X, c_n, z_n, m_n, r_n1_n, m_w_n, p_n, ...
 *     k_d, b_d, g0, i_rw, unlock, hs_rw_max, rw_g1, rw_g2, w_rw_nom,
 * tau_applied)
 *
 * split the state
 *
 * Arguments    : const double X[21]
 *                double tau_applied[9]
 *                double Xdot[21]
 * Return Type  : void
 */
static void bit_propagator(const double X[21], double tau_applied[9],
                           double Xdot[21])
{
  static const double m_w_n[324] = {
      0.0,   0.0,   0.0,   0.0,   0.0,   -0.0,  0.0,   0.0,    0.0,   -0.0,
      0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   -0.0,  0.0,    -0.0,  -0.0,
      0.0,   0.0,   0.0,   0.0,   0.0,   -0.0,  -0.0,  0.0,    0.0,   0.0,
      -0.0,  0.0,   -0.0,  0.0,   0.0,   0.0,   0.0,   0.0,    0.0,   0.0,
      0.0,   -0.0,  0.0,   0.0,   0.0,   -0.0,  0.0,   0.0,    0.0,   0.0,
      0.0,   0.0,   -0.0,  0.0,   -0.0,  -0.0,  0.0,   0.0,    0.0,   0.0,
      0.0,   -0.0,  -0.0,  0.0,   0.0,   0.0,   -0.0,  0.0,    -0.0,  0.0,
      0.0,   0.0,   10.0,  0.0,   0.0,   0.0,   -30.5, -0.0,   0.0,   10.0,
      0.0,   30.5,  0.0,   0.0,   0.0,   0.0,   10.0,  0.0,    -0.0,  0.0,
      -0.0,  30.5,  0.0,   1.0,   0.0,   0.0,   -30.5, -0.0,   -0.0,  0.0,
      1.0,   0.0,   -0.0,  0.0,   -0.0,  0.0,   0.0,   10.0,   0.0,   0.0,
      0.0,   0.0,   0.0,   -0.0,  0.0,   0.0,   0.0,   -0.0,   0.0,   0.0,
      0.0,   0.0,   0.0,   0.0,   -0.0,  0.0,   -0.0,  -0.0,   0.0,   0.0,
      0.0,   0.0,   0.0,   -0.0,  -0.0,  0.0,   0.0,   0.0,    -0.0,  0.0,
      -0.0,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,    0.0,   -0.0,
      0.0,   0.0,   0.0,   -0.0,  0.0,   0.0,   0.0,   0.0,    0.0,   0.0,
      -0.0,  0.0,   -0.0,  -0.0,  0.0,   0.0,   0.0,   0.0,    0.0,   -0.0,
      -0.0,  0.0,   0.0,   0.0,   -0.0,  0.0,   -0.0,  0.0,    0.0,   0.0,
      1.0,   0.0,   0.0,   0.0,   -1.4,  -0.0,  0.0,   1.0,    0.0,   1.4,
      0.0,   0.0,   0.0,   0.0,   1.0,   0.0,   -0.0,  0.0,    -0.0,  1.4,
      0.0,   1.0,   0.0,   0.0,   -1.4,  -0.0,  -0.0,  0.0,    1.0,   0.0,
      -0.0,  0.0,   -0.0,  0.0,   0.0,   1.0,   350.0, 0.0,    0.0,   0.0,
      0.0,   -0.0,  0.0,   350.0, 0.0,   -0.0,  0.0,   0.0,    0.0,   0.0,
      350.0, 0.0,   -0.0,  0.0,   -0.0,  -0.0,  0.0,   150.35, -0.32, -0.07,
      0.0,   -0.0,  -0.0,  -0.32, 79.89, 0.85,  -0.0,  0.0,    -0.0,  -0.07,
      0.85,  117.0, 73.0,  0.0,   0.0,   0.0,   0.0,   -0.0,   0.0,   73.0,
      0.0,   -0.0,  0.0,   0.0,   0.0,   0.0,   73.0,  0.0,    -0.0,  0.0,
      -0.0,  -0.0,  0.0,   30.09, 0.0,   0.0,   0.0,   -0.0,   -0.0,  0.0,
      23.17, 0.0,   -0.0,  0.0,   -0.0,  0.0,   0.0,   19.8,   150.0, 0.0,
      0.0,   0.0,   0.0,   -0.0,  0.0,   150.0, 0.0,   -0.0,   0.0,   0.0,
      0.0,   0.0,   150.0, 0.0,   -0.0,  0.0,   -0.0,  -0.0,   0.0,   33.71,
      0.0,   0.0,   0.0,   -0.0,  -0.0,  0.0,   25.53, 10.5,   -0.0,  0.0,
      -0.0,  0.0,   10.5,  22.79};
  static const double c_n[27] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -30.5,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.4,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static const double r_n1_n[27] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, -61.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.4, 0.0, 0.0,   0.0};
  static const double dv[9] = {0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0,
                               0.0017453292519943296,
                               0.0,
                               12.281626381649737,
                               12.281626381649737};
  static const double b[3] = {0.0, 0.0, -9.72};
  static const short iv[9] = {0, 0, 10, 0, 0, 1, 350, 73, 150};
  static const signed char p_n[54] = {0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
                                      0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                                      1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1,
                                      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0};
  static const signed char z_n[27] = {0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1,
                                      0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0};
  double T_n[324];
  double C_n[81];
  double C_n_rate[81];
  double T_nj[36];
  double b_T_n[36];
  double b_T_ni[36];
  double b_dVdtheta_i[27];
  double b_r_tmp[27];
  double r_tmp[27];
  double s7[27];
  double Pot[9];
  double b_C_n[9];
  double dC_10[9];
  double dVdtheta_i[9];
  double dtheta[9];
  double t1[9];
  double vec[3];
  double M_ij;
  double d;
  double d1;
  double d2;
  double d_hs_idx_0_tmp;
  double d_hs_idx_2;
  double m_i;
  double tau_rw;
  int b_i;
  int b_j;
  int b_vec_tmp;
  int i;
  int i2;
  int i3;
  int i4;
  int idxA1j;
  int idxAjj;
  int idxAjjp1;
  int info;
  int j;
  int jmax;
  int k;
  int n;
  int vec_tmp;
  boolean_T x[3];
  boolean_T exitg1;
  boolean_T y;
  /* 'bit_propagator:4' theta = X(10:18); */
  /* 'bit_propagator:5' dtheta = X(1:9); */
  /* 'bit_propagator:6' hs = X(19:21); */
  /* extract RW torque. */
  /* 'bit_propagator:9' tau_rw = tau_applied(7); */
  tau_rw = tau_applied[6];
  /* 'bit_propagator:10' tau_applied(7) = 0; */
  tau_applied[6] = 0.0;
  /*      tau_applied = zeros(9,1); */
  /*     tau_rw = tau_applied(7); */
  /*     %% */
  /* 'bit_propagator:16' Pot = compute_potential_energy_term(theta, c_n, z_n,
   * m_n, r_n1_n, g0); */
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
  /* 'compute_potential_energy_term:16' for n = 1:ndof */
  b_C_n[0] = -0.0;
  b_C_n[4] = -0.0;
  b_C_n[8] = -0.0;
  for (n = 0; n < 9; n++) {
    dtheta[n] = X[n];
    Pot[n] = 0.0;
    /* 'compute_potential_energy_term:17' C_n(:,:,n) = axis2rot(z_n(:,n),
     * theta(n)); */
    vec_tmp = z_n[3 * n];
    vec[0] = vec_tmp;
    b_vec_tmp = z_n[3 * n + 1];
    vec[1] = b_vec_tmp;
    jmax = z_n[3 * n + 2];
    vec[2] = jmax;
    axis2rot(vec, X[n + 9], *(double(*)[9]) & C_n[9 * n]);
    /* Partial of Rot_n with respect to theta_n */
    /* 'compute_potential_energy_term:19' C_n_rate(:,:,n) = -1 * xmat(z_n(:,n))
     * * C_n(:,:,n); */
    /* 'xmat:2' mat = [        0, -vec(3),  vec(2); */
    /* 'xmat:3'             vec(3),         0, -vec(1); */
    /* 'xmat:4'            -vec(2),  vec(1),         0]; */
    b_C_n[3] = jmax;
    b_C_n[6] = -(double)b_vec_tmp;
    b_C_n[1] = -(double)jmax;
    b_C_n[7] = vec_tmp;
    b_C_n[2] = b_vec_tmp;
    b_C_n[5] = -(double)vec_tmp;
    for (i = 0; i < 3; i++) {
      i2 = (int)b_C_n[i];
      b_vec_tmp = (int)b_C_n[i + 3];
      info = (int)b_C_n[i + 6];
      for (idxA1j = 0; idxA1j < 3; idxA1j++) {
        jmax = 3 * idxA1j + 9 * n;
        C_n_rate[(i + 3 * idxA1j) + 9 * n] =
            ((double)i2 * C_n[jmax] + (double)b_vec_tmp * C_n[jmax + 1]) +
            (double)info * C_n[jmax + 2];
      }
    }
  }
  /*  Compute potential energy term. First loop is cycling through each */
  /*  koints contribution. */
  /* 'compute_potential_energy_term:24' for n = 1:ndof */
  for (n = 0; n < 9; n++) {
    /* 'compute_potential_energy_term:25' m_i = 0; */
    m_i = 0.0;
    /* 'compute_potential_energy_term:26' dVdtheta_i = zeros(ndof,1); */
    memset(&dVdtheta_i[0], 0, 9U * sizeof(double));
    /*  mass of remaining link (ex. 7 + 8 + 9) is totla mass at OF joint */
    /* 'compute_potential_energy_term:29' for q = n:ndof */
    i = 8 - n;
    for (jmax = 0; jmax <= i; jmax++) {
      /* 'compute_potential_energy_term:30' m_i = m_i + m_n(q); */
      m_i += (double)iv[n + jmax];
    }
    /* 'compute_potential_energy_term:33' if (m_i ~= 0) */
    if (m_i != 0.0) {
      /* 'compute_potential_energy_term:34' t1 = zeros(ndof,1); */
      memset(&t1[0], 0, 9U * sizeof(double));
      /* 'compute_potential_energy_term:35' t2 = t1; */
      /* Cycling through all joints up to joint n, the jint we are */
      /* currently calculating the contribution from */
      /*  */
      /* terms up to n-1 links.  */
      /* 'compute_potential_energy_term:41' for j = 1:n-1 */
      for (j = 0; j < n; j++) {
        /* 'compute_potential_energy_term:42' dC_10 = eye(3); */
        memset(&dC_10[0], 0, 9U * sizeof(double));
        dC_10[0] = 1.0;
        dC_10[4] = 1.0;
        dC_10[8] = 1.0;
        /* 'compute_potential_energy_term:44' for k = 1:n-1 */
        for (k = 0; k < n; k++) {
          /* This multiplies the rotation matricies successively */
          /*  until the link j where the rate is inserted instead */
          /* 'compute_potential_energy_term:47' if (k == j) */
          if (k == j) {
            /* 'compute_potential_energy_term:48' dC_10 = C_n_rate(:,:,k) *
             * dC_10; */
            for (i = 0; i < 3; i++) {
              i2 = i + 9 * k;
              for (b_vec_tmp = 0; b_vec_tmp < 3; b_vec_tmp++) {
                b_C_n[i + 3 * b_vec_tmp] =
                    (C_n_rate[i2] * dC_10[3 * b_vec_tmp] +
                     C_n_rate[i2 + 3] * dC_10[3 * b_vec_tmp + 1]) +
                    C_n_rate[i2 + 6] * dC_10[3 * b_vec_tmp + 2];
              }
            }
            memcpy(&dC_10[0], &b_C_n[0], 9U * sizeof(double));
          } else {
            /* 'compute_potential_energy_term:49' else */
            /* 'compute_potential_energy_term:50' dC_10 = C_n(:,:,k) * dC_10; */
            for (i = 0; i < 3; i++) {
              i2 = i + 9 * k;
              for (b_vec_tmp = 0; b_vec_tmp < 3; b_vec_tmp++) {
                b_C_n[i + 3 * b_vec_tmp] =
                    (C_n[i2] * dC_10[3 * b_vec_tmp] +
                     C_n[i2 + 3] * dC_10[3 * b_vec_tmp + 1]) +
                    C_n[i2 + 6] * dC_10[3 * b_vec_tmp + 2];
              }
            }
            memcpy(&dC_10[0], &b_C_n[0], 9U * sizeof(double));
          }
        }
        /* 'compute_potential_energy_term:53' t1(j) = r_n1_n(:,n)' * dC_10 * g0;
         */
        M_ij = 0.0;
        d = r_n1_n[3 * n];
        d1 = r_n1_n[3 * n + 1];
        d2 = r_n1_n[3 * n + 2];
        for (i = 0; i < 3; i++) {
          M_ij += ((d * dC_10[3 * i] + d1 * dC_10[3 * i + 1]) +
                   d2 * dC_10[3 * i + 2]) *
                  b[i];
        }
        t1[j] = M_ij;
      }
      /* %% PE terms that go from 1:n */
      /* 'compute_potential_energy_term:56' for j = 1:n */
      d = c_n[3 * n];
      d1 = c_n[3 * n + 1];
      d2 = c_n[3 * n + 2];
      for (j = 0; j <= n; j++) {
        /* 'compute_potential_energy_term:57' dC_10 = eye(3); */
        memset(&dC_10[0], 0, 9U * sizeof(double));
        dC_10[0] = 1.0;
        dC_10[4] = 1.0;
        dC_10[8] = 1.0;
        /* 'compute_potential_energy_term:59' for k = 1:n */
        for (k = 0; k <= n; k++) {
          /* This multiplies the rotation matricies successively */
          /*  until the link j is reached in which case the rate */
          /*  is multiplied by the overall rotation */
          /* 'compute_potential_energy_term:63' if (k == j) */
          if (k == j) {
            /* 'compute_potential_energy_term:64' dC_10 = C_n_rate(:,:,k) *
             * dC_10; */
            for (i = 0; i < 3; i++) {
              i2 = i + 9 * k;
              for (b_vec_tmp = 0; b_vec_tmp < 3; b_vec_tmp++) {
                b_C_n[i + 3 * b_vec_tmp] =
                    (C_n_rate[i2] * dC_10[3 * b_vec_tmp] +
                     C_n_rate[i2 + 3] * dC_10[3 * b_vec_tmp + 1]) +
                    C_n_rate[i2 + 6] * dC_10[3 * b_vec_tmp + 2];
              }
            }
            memcpy(&dC_10[0], &b_C_n[0], 9U * sizeof(double));
          } else {
            /* 'compute_potential_energy_term:65' else */
            /* 'compute_potential_energy_term:66' dC_10 = C_n(:,:,k) * dC_10; */
            for (i = 0; i < 3; i++) {
              i2 = i + 9 * k;
              for (b_vec_tmp = 0; b_vec_tmp < 3; b_vec_tmp++) {
                b_C_n[i + 3 * b_vec_tmp] =
                    (C_n[i2] * dC_10[3 * b_vec_tmp] +
                     C_n[i2 + 3] * dC_10[3 * b_vec_tmp + 1]) +
                    C_n[i2 + 6] * dC_10[3 * b_vec_tmp + 2];
              }
            }
            memcpy(&dC_10[0], &b_C_n[0], 9U * sizeof(double));
          }
        }
        /* dot product */
        /* %%% made change here c_n(n) -> c_n(:,n) */
        /* 'compute_potential_energy_term:71' t2 = c_n(:,n)' * dC_10 * g0; */
        /* 'compute_potential_energy_term:73' dVdtheta_i(j) = -m_i*t1(j)-t2; */
        M_ij = 0.0;
        for (i = 0; i < 3; i++) {
          M_ij += ((d * dC_10[3 * i] + d1 * dC_10[3 * i + 1]) +
                   d2 * dC_10[3 * i + 2]) *
                  b[i];
        }
        dVdtheta_i[j] = -m_i * t1[j] - M_ij;
      }
    }
    /* 'compute_potential_energy_term:76' Pot = Pot + dVdtheta_i; */
    for (i = 0; i < 9; i++) {
      Pot[i] += dVdtheta_i[i];
    }
  }
  /* 'bit_propagator:18' spring = k_d.*theta; */
  /* 'bit_propagator:19' damp = b_d.*dtheta; */
  /* place holder */
  /* 'bit_propagator:22' [R,r, d_hs, w_piv] = RW_terms(theta, dtheta, z_n,i_rw,
   * hs, tau_rw, hs_rw_max, ... */
  /* 'bit_propagator:23'         rw_g1, rw_g2, w_rw_nom); */
  /* UNTITLED Summary of this function goes here */
  /*    Detailed explanation goes here */
  /* calculate the mapping matrix from dtheta to omega */
  /* 'RW_terms:7' s7 = zeros(3,9); */
  memset(&s7[0], 0, 27U * sizeof(double));
  /* 'RW_terms:8' for i = 1:7 */
  for (b_i = 0; b_i < 7; b_i++) {
    signed char i1;
    /* 'RW_terms:9' Cn = axis2rot(z_n(:,i), theta(i)); */
    /* 'RW_terms:10' s7(:,i) = z_n(:,i); */
    i1 = z_n[3 * b_i];
    vec[0] = i1;
    s7[3 * b_i] = i1;
    i = 3 * b_i + 1;
    i1 = z_n[i];
    vec[1] = i1;
    s7[i] = i1;
    i = 3 * b_i + 2;
    i1 = z_n[i];
    vec[2] = i1;
    s7[i] = i1;
    axis2rot(vec, X[b_i + 9], dVdtheta_i);
    /* 'RW_terms:11' s7 = Cn*s7; */
    for (i = 0; i < 3; i++) {
      d = dVdtheta_i[i];
      d1 = dVdtheta_i[i + 3];
      d2 = dVdtheta_i[i + 6];
      for (i2 = 0; i2 < 9; i2++) {
        b_dVdtheta_i[i + 3 * i2] =
            (d * s7[3 * i2] + d1 * s7[3 * i2 + 1]) + d2 * s7[3 * i2 + 2];
      }
    }
    memcpy(&s7[0], &b_dVdtheta_i[0], 27U * sizeof(double));
  }
  /* 'RW_terms:15' d_hs = tau_rw*z_n(:,7); */
  /* 'RW_terms:17' if hs(3) >= hs_rw_max */
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
  if (y) {
    /* 'RW_terms:18' if d_hs(3) > 0 */
    if (tau_rw > 0.0) {
      /* 'RW_terms:19' d_hs(3) = 0; */
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
    if (y && (tau_rw < 0.0)) {
      /* 'RW_terms:21' elseif hs(3) <= -hs_rw_max */
      /* 'RW_terms:22' if d_hs(3) < 0 */
      /* 'RW_terms:23' d_hs(3) = 0; */
      d_hs_idx_2 = 0.0;
    }
  }
  /* 'RW_terms:27' r = s7'*d_hs; */
  for (i = 0; i < 3; i++) {
    for (i2 = 0; i2 < 9; i2++) {
      b_dVdtheta_i[i2 + 9 * i] = s7[i + 3 * i2];
    }
  }
  /* 'RW_terms:29' R = -s7' * xmat(hs) * s7 * dtheta; */
  /* 'xmat:2' mat = [        0, -vec(3),  vec(2); */
  /* 'xmat:3'             vec(3),         0, -vec(1); */
  /* 'xmat:4'            -vec(2),  vec(1),         0]; */
  /* 'RW_terms:31' w_piv = -rw_g1*((hs(3)/i_rw(3,3))-w_rw_nom) - rw_g2*tau_rw;
   */
  dtheta[5] =
      -0.034 * (X[20] / 9.0 - 3.1415926535897931) - 2.3539622976350807 * tau_rw;
  /*  calculate the ddtheta resulting from desired  */
  /* calculate joint torques from gravity elasticity and damnping according */
  /* to eq 3.37 */
  /* 'bit_propagator:29' torques = tau_applied - (Pot + spring + damp + R + r);
   */
  /* 'bit_propagator:31' M = compute_mass_matrix(theta, z_n, r_n1_n, m_w_n,
   * p_n); */
  /* Compute_Mass_Matrix computes the mass matrix of the 9 state system using */
  /* the angles theta to calculate the interbody transformation matrices and */
  /* the axis of rotations. */
  /*  Initialize mass matrix */
  /* 'compute_mass_matrix:7' mass_mat = (zeros(9,9)); */
  memset(&C_n[0], 0, 81U * sizeof(double));
  /*  memory to store the interbody transformations */
  /* 'compute_mass_matrix:10' T_n = (zeros(6,6,9)); */
  /* 'compute_mass_matrix:12' T_ni = (zeros(6,6)); */
  /* 'compute_mass_matrix:13' T_nj = T_ni; */
  /* 'compute_mass_matrix:15' J_ni = (zeros(6,1)); */
  /* 'compute_mass_matrix:16' J_nj = J_ni; */
  /*  equation 3.8: Interbody transformations for each frame */
  /* 'compute_mass_matrix:19' for i = 1:9 */
  b_C_n[0] = 0.0;
  b_C_n[4] = 0.0;
  b_C_n[8] = 0.0;
  for (b_i = 0; b_i < 9; b_i++) {
    /* 'compute_mass_matrix:20' C_n = axis2rot(z_n(:,i), theta(i)); */
    vec[0] = z_n[3 * b_i];
    vec_tmp = 3 * b_i + 1;
    vec[1] = z_n[vec_tmp];
    b_vec_tmp = 3 * b_i + 2;
    vec[2] = z_n[b_vec_tmp];
    axis2rot(vec, X[b_i + 9], dVdtheta_i);
    /* 'compute_mass_matrix:21' off_term = xmat(r_n1_n(:,i)); */
    vec[0] = r_n1_n[3 * b_i];
    vec[1] = r_n1_n[vec_tmp];
    vec[2] = r_n1_n[b_vec_tmp];
    /* 'xmat:2' mat = [        0, -vec(3),  vec(2); */
    /* 'xmat:3'             vec(3),         0, -vec(1); */
    /* 'xmat:4'            -vec(2),  vec(1),         0]; */
    /* 'compute_mass_matrix:22' off_term = -1*C_n*off_term; */
    /* 'compute_mass_matrix:23' T_n(:,:,i) = ([C_n, off_term; zeros(3,3), C_n]);
     */
    b_C_n[3] = -vec[2];
    b_C_n[6] = vec[1];
    b_C_n[1] = vec[2];
    b_C_n[7] = -vec[0];
    b_C_n[2] = -vec[1];
    b_C_n[5] = vec[0];
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dC_10[i + 3 * i2] = (-dVdtheta_i[i] * b_C_n[3 * i2] +
                             -dVdtheta_i[i + 3] * b_C_n[3 * i2 + 1]) +
                            -dVdtheta_i[i + 6] * b_C_n[3 * i2 + 2];
        T_n[(i2 + 6 * i) + 36 * b_i] = dVdtheta_i[i2 + 3 * i];
      }
    }
    for (i = 0; i < 3; i++) {
      jmax = 6 * (i + 3) + 36 * b_i;
      T_n[jmax] = dC_10[3 * i];
      vec_tmp = 6 * i + 36 * b_i;
      T_n[vec_tmp + 3] = 0.0;
      T_n[jmax + 3] = dVdtheta_i[3 * i];
      b_vec_tmp = 3 * i + 1;
      T_n[jmax + 1] = dC_10[b_vec_tmp];
      T_n[vec_tmp + 4] = 0.0;
      T_n[jmax + 4] = dVdtheta_i[b_vec_tmp];
      b_vec_tmp = 3 * i + 2;
      T_n[jmax + 2] = dC_10[b_vec_tmp];
      T_n[vec_tmp + 5] = 0.0;
      T_n[jmax + 5] = dVdtheta_i[b_vec_tmp];
    }
  }
  /*  Generate the mass matrix */
  /*  Eq 3.12-3.13 */
  /* 'compute_mass_matrix:28' for i = 1:9 */
  for (b_i = 0; b_i < 9; b_i++) {
    /* 'compute_mass_matrix:29' for j = i:9 */
    i = 8 - b_i;
    for (j = 0; j <= i; j++) {
      idxAjj = b_i + j;
      /* 'compute_mass_matrix:30' M_ij = 0; */
      M_ij = 0.0;
      /* 'compute_mass_matrix:31' for n = j:9 */
      i2 = 8 - idxAjj;
      if (i2 >= 0) {
        i3 = idxAjj - b_i;
      }
      for (n = 0; n <= i2; n++) {
        double T_ni[6];
        idxAjjp1 = idxAjj + n;
        /* 'compute_mass_matrix:32' T_ni = (eye(6)); */
        memset(&b_T_ni[0], 0, 36U * sizeof(double));
        for (k = 0; k < 6; k++) {
          b_T_ni[k + 6 * k] = 1.0;
        }
        /* 'compute_mass_matrix:33' T_nj = T_ni; */
        memcpy(&T_nj[0], &b_T_ni[0], 36U * sizeof(double));
        /* 'compute_mass_matrix:35' for k = i+1:j */
        for (k = 0; k < i3; k++) {
          /*                  tic */
          /* 'compute_mass_matrix:37' T_ni = T_n(:,:,k)*T_ni; */
          jmax = b_i + k;
          for (b_vec_tmp = 0; b_vec_tmp < 6; b_vec_tmp++) {
            for (info = 0; info < 6; info++) {
              d = 0.0;
              for (idxA1j = 0; idxA1j < 6; idxA1j++) {
                d += T_n[(b_vec_tmp + 6 * idxA1j) + 36 * (jmax + 1)] *
                     b_T_ni[idxA1j + 6 * info];
              }
              b_T_n[b_vec_tmp + 6 * info] = d;
            }
          }
          memcpy(&b_T_ni[0], &b_T_n[0], 36U * sizeof(double));
          /*                  toc */
          /*                  tic */
          /*                  T_ni = mtimes(T_n(:,:,k),T_ni); */
          /*                  toc */
        }
        /* 'compute_mass_matrix:44' for k = j+1:n */
        b_vec_tmp = idxAjjp1 - idxAjj;
        for (k = 0; k < b_vec_tmp; k++) {
          /* 'compute_mass_matrix:45' T_nj = T_n(:,:,k) * T_nj; */
          vec_tmp = idxAjj + k;
          for (info = 0; info < 6; info++) {
            for (idxA1j = 0; idxA1j < 6; idxA1j++) {
              d = 0.0;
              for (jmax = 0; jmax < 6; jmax++) {
                d += T_n[(info + 6 * jmax) + 36 * (vec_tmp + 1)] *
                     T_nj[jmax + 6 * idxA1j];
              }
              b_T_n[info + 6 * idxA1j] = d;
            }
          }
          memcpy(&T_nj[0], &b_T_n[0], 36U * sizeof(double));
          /*                  T_nj = mtimes(T_n(:,:,k),T_nj); */
        }
        /* 'compute_mass_matrix:48' T_ni = T_nj*T_ni; */
        for (b_vec_tmp = 0; b_vec_tmp < 6; b_vec_tmp++) {
          for (info = 0; info < 6; info++) {
            d = 0.0;
            for (idxA1j = 0; idxA1j < 6; idxA1j++) {
              d += T_nj[b_vec_tmp + 6 * idxA1j] * b_T_ni[idxA1j + 6 * info];
            }
            b_T_n[b_vec_tmp + 6 * info] = d;
          }
        }
        memcpy(&b_T_ni[0], &b_T_n[0], 36U * sizeof(double));
        /*              T_ni = mtimes(T_nj,T_ni); */
        /* 'compute_mass_matrix:51' J_ni = (T_ni) * p_n(:,i); */
        /* 'compute_mass_matrix:52' J_nj = (T_nj) * p_n(:,j); */
        /* 'compute_mass_matrix:54' add = (J_ni' * m_w_n(:,:,n) * J_nj); */
        /* 'compute_mass_matrix:55' M_ij = M_ij + add; */
        for (b_vec_tmp = 0; b_vec_tmp < 6; b_vec_tmp++) {
          d = 0.0;
          for (info = 0; info < 6; info++) {
            d += b_T_ni[b_vec_tmp + 6 * info] * (double)p_n[info + 6 * b_i];
          }
          T_ni[b_vec_tmp] = d;
        }
        m_i = 0.0;
        for (b_vec_tmp = 0; b_vec_tmp < 6; b_vec_tmp++) {
          d = 0.0;
          d1 = 0.0;
          for (info = 0; info < 6; info++) {
            d += T_ni[info] * m_w_n[(info + 6 * b_vec_tmp) + 36 * idxAjjp1];
            d1 += T_nj[b_vec_tmp + 6 * info] * (double)p_n[info + 6 * idxAjj];
          }
          m_i += d * d1;
        }
        M_ij += m_i;
      }
      /* 'compute_mass_matrix:57' mass_mat(i,j) = M_ij; */
      C_n[b_i + 9 * idxAjj] = M_ij;
      /* 'compute_mass_matrix:58' if (i ~= j) */
      if (b_i != idxAjj) {
        /* 'compute_mass_matrix:59' mass_mat(j,i) = M_ij; */
        C_n[idxAjj + 9 * b_i] = M_ij;
      }
    }
  }
  /*      Mass = mass_mat_func(theta); */
  /* 'bit_propagator:34' M_decomp = chol(M); */
  info = -2;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 9)) {
    idxA1j = j * 9;
    idxAjj = idxA1j + j;
    m_i = 0.0;
    if (j >= 1) {
      for (k = 0; k < j; k++) {
        M_ij = C_n[idxA1j + k];
        m_i += M_ij * M_ij;
      }
    }
    m_i = C_n[idxAjj] - m_i;
    if (m_i > 0.0) {
      m_i = sqrt(m_i);
      C_n[idxAjj] = m_i;
      if (j + 1 < 9) {
        vec_tmp = idxA1j + 10;
        idxAjjp1 = idxAjj + 10;
        if (j != 0) {
          i = (idxA1j + 9 * (7 - j)) + 10;
          for (b_vec_tmp = vec_tmp; b_vec_tmp <= i; b_vec_tmp += 9) {
            M_ij = 0.0;
            i2 = (b_vec_tmp + j) - 1;
            for (jmax = b_vec_tmp; jmax <= i2; jmax++) {
              M_ij += C_n[jmax - 1] * C_n[(idxA1j + jmax) - b_vec_tmp];
            }
            jmax =
                (idxAjj + div_nde_s32_floor((b_vec_tmp - idxA1j) - 10) * 9) + 9;
            C_n[jmax] += -M_ij;
          }
        }
        m_i = 1.0 / m_i;
        i = (idxAjj + 9 * (7 - j)) + 10;
        for (k = idxAjjp1; k <= i; k += 9) {
          C_n[k - 1] *= m_i;
        }
      }
      j++;
    } else {
      C_n[idxAjj] = m_i;
      info = j - 1;
      exitg1 = true;
    }
  }
  if (info + 2 == 0) {
    jmax = 7;
  } else {
    jmax = info - 1;
  }
#pragma omp parallel for num_threads(                                          \
    8 > omp_get_max_threads() ? omp_get_max_threads() : 8) private(i4)

  for (b_j = 0; b_j <= jmax; b_j++) {
    i4 = b_j + 2;
    if (i4 <= jmax + 2) {
      memset(&C_n[(b_j * 9 + i4) + -1], 0, ((jmax - i4) + 3) * sizeof(double));
    }
  }
  /* 'bit_propagator:35' ddtheta = M_decomp\(M_decomp'\torques); */
  /* 'bit_propagator:37' ddtheta = ddtheta.*unlock; */
  /*  set pivot speed in dtheta... */
  /* 'bit_propagator:40' dtheta(6) = w_piv; */
  /* 'bit_propagator:41' Xdot = [ddtheta; dtheta; d_hs;]; */
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
    for (i2 = 0; i2 < 3; i2++) {
      b_r_tmp[i + 9 * i2] =
          (d * b_C_n[3 * i2] + d1 * b_C_n[3 * i2 + 1]) + d2 * b_C_n[3 * i2 + 2];
    }
    d = 0.0;
    d1 = (b_dVdtheta_i[i] * d_hs_idx_0_tmp +
          b_dVdtheta_i[i + 9] * d_hs_idx_0_tmp) +
         b_dVdtheta_i[i + 18] * d_hs_idx_2;
    d2 = b_r_tmp[i];
    m_i = b_r_tmp[i + 9];
    M_ij = b_r_tmp[i + 18];
    for (i2 = 0; i2 < 9; i2++) {
      d += ((d2 * s7[3 * i2] + m_i * s7[3 * i2 + 1]) + M_ij * s7[3 * i2 + 2]) *
           X[i2];
      C_n_rate[i2 + 9 * i] = C_n[i + 9 * i2];
    }
    d += (Pot[i] + dv[i] * X[i + 9]) + 0.0 * X[i];
    Pot[i] = d1;
    d = tau_applied[i] - (d + d1);
    dVdtheta_i[i] = d;
  }
  mldivide(C_n_rate, dVdtheta_i);
  mldivide(C_n, dVdtheta_i);
  for (b_i = 0; b_i < 9; b_i++) {
    Xdot[b_i] = dVdtheta_i[b_i];
    Xdot[b_i + 9] = dtheta[b_i];
  }
  Xdot[18] = d_hs_idx_0_tmp;
  Xdot[19] = d_hs_idx_0_tmp;
  Xdot[20] = d_hs_idx_2;
}

/*
 * Arguments    : int numerator
 * Return Type  : int
 */
static int div_nde_s32_floor(int numerator)
{
  int b_numerator;
  if ((numerator < 0) && (numerator % 9 != 0)) {
    b_numerator = -1;
  } else {
    b_numerator = 0;
  }
  return numerator / 9 + b_numerator;
}

/*
 * Arguments    : const double A[81]
 *                double B[9]
 * Return Type  : void
 */
static void mldivide(const double A[81], double B[9])
{
  double b_A[81];
  int A_tmp;
  int a;
  int i;
  int j;
  int jA;
  int jp1j;
  int k;
  signed char ipiv[9];
  memcpy(&b_A[0], &A[0], 81U * sizeof(double));
  for (i = 0; i < 9; i++) {
    ipiv[i] = (signed char)(i + 1);
  }
  for (j = 0; j < 8; j++) {
    double smax;
    int b_tmp;
    int mmj_tmp;
    signed char i1;
    mmj_tmp = 7 - j;
    b_tmp = j * 10;
    jp1j = b_tmp + 2;
    jA = 9 - j;
    a = 0;
    smax = fabs(b_A[b_tmp]);
    for (k = 2; k <= jA; k++) {
      double s;
      s = fabs(b_A[(b_tmp + k) - 1]);
      if (s > smax) {
        a = k - 1;
        smax = s;
      }
    }
    if (b_A[b_tmp + a] != 0.0) {
      if (a != 0) {
        jA = j + a;
        ipiv[j] = (signed char)(jA + 1);
        for (k = 0; k < 9; k++) {
          a = j + k * 9;
          smax = b_A[a];
          A_tmp = jA + k * 9;
          b_A[a] = b_A[A_tmp];
          b_A[A_tmp] = smax;
        }
      }
      i = (b_tmp - j) + 9;
      for (a = jp1j; a <= i; a++) {
        b_A[a - 1] /= b_A[b_tmp];
      }
    }
    jA = b_tmp;
    for (A_tmp = 0; A_tmp <= mmj_tmp; A_tmp++) {
      smax = b_A[(b_tmp + A_tmp * 9) + 9];
      if (smax != 0.0) {
        i = jA + 11;
        a = (jA - j) + 18;
        for (jp1j = i; jp1j <= a; jp1j++) {
          b_A[jp1j - 1] += b_A[((b_tmp + jp1j) - jA) - 10] * -smax;
        }
      }
      jA += 9;
    }
    i1 = ipiv[j];
    if (i1 != j + 1) {
      smax = B[j];
      B[j] = B[i1 - 1];
      B[i1 - 1] = smax;
    }
  }
  for (k = 0; k < 9; k++) {
    jA = 9 * k;
    if (B[k] != 0.0) {
      i = k + 2;
      for (a = i; a < 10; a++) {
        B[a - 1] -= B[k] * b_A[(a + jA) - 1];
      }
    }
  }
  for (k = 8; k >= 0; k--) {
    jA = 9 * k;
    if (B[k] != 0.0) {
      B[k] /= b_A[k + jA];
      for (a = 0; a < k; a++) {
        B[a] -= B[k] * b_A[a + jA];
      }
    }
  }
}

/*
 * function [y_all] = bit_one_step(x0, tau_applied, ...
 *                                              dt, tf)
 *
 * Run initialization script
 *
 * Arguments    : const double x0[21]
 *                const double tau_applied[9]
 *                double dt
 *                double tf
 *                emxArray_real_T *y_all
 * Return Type  : void
 */
void bit_one_step(const double x0[21], const double tau_applied[9], double dt,
                  double tf, emxArray_real_T *y_all)
{
  emxArray_real_T *y;
  double y_true[21];
  double b_tau_applied[9];
  double ndbl;
  double *y_all_data;
  int i;
  int k;
  int n;
  int nm1d2;
  if (!isInitialized_bit_one_step) {
    bit_one_step_initialize();
  }
  /* 'bit_one_step:5' [ndof, g0 r_n1_n, z_n, p_n, m_n, c_n, ... */
  /* 'bit_one_step:6'     i_n, m_w_n,  i_rw, bear_k_cst, bear_c_cst, k_d, b_d,
   * ... */
  /* 'bit_one_step:7'     w_rw_max, w_rw_nom, hs_rw, hs_rw_max, ... */
  /* 'bit_one_step:8'      unlock, latency, fs, fs_ekf, rw_g1, rw_g2] =
   * init_func(); */
  /*     %% Setup Simulation */
  /*  initial conditions, state is dtheta; theta */
  /* 'bit_one_step:11' y_true = x0; */
  memcpy(&y_true[0], &x0[0], 21U * sizeof(double));
  /*  Sim Parameters */
  /* 'bit_one_step:14' t_vec = dt:dt:tf; */
  emxInit_real_T(&y, 2);
  if (rtIsNaN(dt) || rtIsNaN(dt) || rtIsNaN(tf)) {
    nm1d2 = y->size[0] * y->size[1];
    y->size[1] = 1;
    emxEnsureCapacity_real_T(y, nm1d2);
  } else if ((dt == 0.0) || ((dt < tf) && (dt < 0.0)) ||
             ((tf < dt) && (dt > 0.0))) {
    y->size[1] = 0;
  } else if ((rtIsInf(dt) || rtIsInf(tf)) && (rtIsInf(dt) || (dt == tf))) {
    nm1d2 = y->size[0] * y->size[1];
    y->size[1] = 1;
    emxEnsureCapacity_real_T(y, nm1d2);
  } else if (rtIsInf(dt)) {
    nm1d2 = y->size[0] * y->size[1];
    y->size[1] = 1;
    emxEnsureCapacity_real_T(y, nm1d2);
  } else if (floor(dt) == dt) {
    nm1d2 = y->size[0] * y->size[1];
    y->size[1] = (int)((tf - dt) / dt) + 1;
    emxEnsureCapacity_real_T(y, nm1d2);
  } else {
    double apnd;
    double cdiff;
    ndbl = floor((tf - dt) / dt + 0.5);
    apnd = dt + ndbl * dt;
    if (dt > 0.0) {
      cdiff = apnd - tf;
    } else {
      cdiff = tf - apnd;
    }
    if (fabs(cdiff) < 4.4408920985006262E-16 * fmax(fabs(dt), fabs(tf))) {
      ndbl++;
      apnd = tf;
    } else if (cdiff > 0.0) {
      apnd = dt + (ndbl - 1.0) * dt;
    } else {
      ndbl++;
    }
    if (ndbl >= 0.0) {
      n = (int)ndbl;
    } else {
      n = 0;
    }
    nm1d2 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = n;
    emxEnsureCapacity_real_T(y, nm1d2);
    y_all_data = y->data;
    if (n > 1) {
      y_all_data[n - 1] = apnd;
      nm1d2 = (n - 1) / 2;
      for (k = 0; k <= nm1d2 - 2; k++) {
        ndbl = ((double)k + 1.0) * dt;
        y_all_data[k + 1] = dt + ndbl;
        y_all_data[(n - k) - 2] = apnd - ndbl;
      }
      if (nm1d2 << 1 == n - 1) {
        y_all_data[nm1d2] = (dt + apnd) / 2.0;
      } else {
        ndbl = (double)nm1d2 * dt;
        y_all_data[nm1d2] = dt + ndbl;
        y_all_data[nm1d2 + 1] = apnd - ndbl;
      }
    }
  }
  /*  y_all1 = zeros(18, tf/(dt)); */
  /* 'bit_one_step:17' y_all = zeros(21, length(t_vec)); */
  nm1d2 = y_all->size[0] * y_all->size[1];
  y_all->size[0] = 21;
  y_all->size[1] = y->size[1];
  emxEnsureCapacity_real_T(y_all, nm1d2);
  y_all_data = y_all->data;
  nm1d2 = 21 * y->size[1];
  if (nm1d2 < 800) {
    for (i = 0; i < nm1d2; i++) {
      y_all_data[i] = 0.0;
    }
  } else {
#pragma omp parallel for num_threads(                                          \
    8 > omp_get_max_threads() ? omp_get_max_threads() : 8)

    for (i = 0; i < nm1d2; i++) {
      y_all_data[i] = 0.0;
    }
  }
  /* 'bit_one_step:18' step = 0; */
  /*  sim */
  /* 'bit_one_step:22' for step = 1:length(t_vec) */
  nm1d2 = y->size[1];
  emxFree_real_T(&y);
  for (n = 0; n < nm1d2; n++) {
    double a[21];
    double b_y_true[21];
    double k1[21];
    double k2[21];
    double k3[21];
    /*          fprintf('time is: %f \n', t_vec(step)) */
    /*          if ~mod(t_vec(step), 1000) */
    /*              timeis = t_vec(step) */
    /*          end */
    /*         %% Propagate the system */
    /* RK4 solver */
    /* 'bit_one_step:31' k1 = bit_propagator(y_true, c_n, z_n, m_n, r_n1_n, ...
     */
    /* 'bit_one_step:32'         m_w_n, p_n, k_d, b_d, g0, i_rw, unlock,
     * hs_rw_max, rw_g1, rw_g2, ... */
    /* 'bit_one_step:33'         w_rw_nom, tau_applied) * dt; */
    memcpy(&b_tau_applied[0], &tau_applied[0], 9U * sizeof(double));
    bit_propagator(y_true, b_tau_applied, k1);
    /* 'bit_one_step:35' k2 = bit_propagator(y_true + k1/2, c_n, z_n, m_n,
     * r_n1_n, ... */
    /* 'bit_one_step:36'         m_w_n, p_n, k_d, b_d, g0, i_rw, unlock,
     * hs_rw_max, rw_g1, rw_g2, ... */
    /* 'bit_one_step:37'         w_rw_nom, tau_applied) * dt; */
    for (k = 0; k < 21; k++) {
      ndbl = k1[k] * dt;
      k1[k] = ndbl;
      b_y_true[k] = y_true[k] + ndbl / 2.0;
    }
    memcpy(&b_tau_applied[0], &tau_applied[0], 9U * sizeof(double));
    bit_propagator(b_y_true, b_tau_applied, k2);
    /* 'bit_one_step:39' k3 = bit_propagator(y_true + k2/2, c_n, z_n, m_n,
     * r_n1_n, ... */
    /* 'bit_one_step:40'         m_w_n, p_n, k_d, b_d, g0, i_rw, unlock,
     * hs_rw_max, rw_g1, rw_g2, ... */
    /* 'bit_one_step:41'         w_rw_nom, tau_applied) * dt; */
    for (k = 0; k < 21; k++) {
      ndbl = k2[k] * dt;
      k2[k] = ndbl;
      b_y_true[k] = y_true[k] + ndbl / 2.0;
    }
    memcpy(&b_tau_applied[0], &tau_applied[0], 9U * sizeof(double));
    bit_propagator(b_y_true, b_tau_applied, k3);
    /* 'bit_one_step:43' k4 = bit_propagator(y_true + k3, c_n, z_n, m_n, r_n1_n,
     * ... */
    /* 'bit_one_step:44'         m_w_n, p_n, k_d, b_d, g0, i_rw, unlock,
     * hs_rw_max, rw_g1, rw_g2, ... */
    /* 'bit_one_step:45'         w_rw_nom, tau_applied) * dt; */
    for (k = 0; k < 21; k++) {
      ndbl = k3[k] * dt;
      k3[k] = ndbl;
      b_y_true[k] = y_true[k] + ndbl;
    }
    memcpy(&b_tau_applied[0], &tau_applied[0], 9U * sizeof(double));
    bit_propagator(b_y_true, b_tau_applied, a);
    /* 'bit_one_step:47' tdd = ((k1+2*k2+2*k3+k4)/6); */
    /* 'bit_one_step:48' y_true = y_true + tdd; */
    for (k = 0; k < 21; k++) {
      y_true[k] += (((k1[k] + 2.0 * k2[k]) + 2.0 * k3[k]) + a[k] * dt) / 6.0;
    }
    /* 'bit_one_step:50' th_over = y_true(10:18) > pi; */
    /* 'bit_one_step:51' th_under = y_true(10:18) < -pi; */
    /* 'bit_one_step:52' y_true(10:18) = y_true(10:18) + 2*pi*th_under -
     * 2*pi*th_over; */
    for (k = 0; k < 9; k++) {
      ndbl = y_true[k + 9];
      y_true[k + 9] =
          (ndbl + 6.2831853071795862 * (double)(ndbl < -3.1415926535897931)) -
          6.2831853071795862 * (double)(ndbl > 3.1415926535897931);
    }
    /* 'bit_one_step:54' y_all(:,step) = y_true; */
    for (k = 0; k < 21; k++) {
      y_all_data[k + 21 * n] = y_true[k];
    }
    /* 'bit_one_step:55' fprintf('current state:  %0.10f  %0.10f %0.10f %0.10f
     * %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f
     * %0.10f %0.10f %0.10f %0.10f  %0.10f %0.10f %0.10f \n', ... */
    /* 'bit_one_step:56'             y_true(1), y_true(2), y_true(3), y_true(4),
     * y_true(5), y_true(6),... */
    /* 'bit_one_step:57'              y_true(7), y_true(8), y_true(9),
     * y_true(10), y_true(11), y_true(12),... */
    /* 'bit_one_step:58'               y_true(13), y_true(14), y_true(15),
     * y_true(16), y_true(17), y_true(18),... */
    /* 'bit_one_step:59'                y_true(19), y_true(20), y_true(21)); */
    printf("current state:  %0.10f  %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f "
           "%0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f %0.10f"
           " %0.10f %0.10f  %0.10f %0.10f %0.10f \n",
           y_true[0], y_true[1], y_true[2], y_true[3], y_true[4], y_true[5],
           y_true[6], y_true[7], y_true[8], y_true[9], y_true[10], y_true[11],
           y_true[12], y_true[13], y_true[14], y_true[15], y_true[16],
           y_true[17], y_true[18], y_true[19], y_true[20]);
    fflush(stdout);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void bit_one_step_initialize(void)
{
  omp_init_nest_lock(&bit_one_step_nestLockGlobal);
  isInitialized_bit_one_step = true;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void bit_one_step_terminate(void)
{
  omp_destroy_nest_lock(&bit_one_step_nestLockGlobal);
  isInitialized_bit_one_step = false;
}

/*
 * File trailer for bit_one_step.c
 *
 * [EOF]
 */
