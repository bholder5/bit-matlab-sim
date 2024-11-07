/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_libbitonestep_api.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 23-Sep-2024 11:45:57
 */

#ifndef _CODER_LIBBITONESTEP_API_H
#define _CODER_LIBBITONESTEP_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void bit_one_step(real_T x0[21], real_T tau_applied[9], real_T unlock[9],
                  real_T w_piv, boolean_T piv_flag, real_T dt,
                  uint16_T num_steps, real_T tau_max_piv, real_T thet_pit_nom,
                  real_T x_flex0[104], real_T tau_flex[5],
                  boolean_T flexure_flag, boolean_T sb_flag, real_T y_true[21],
                  real_T y_flex[104]);

void bit_one_step_api(const mxArray *const prhs[13], int32_T nlhs,
                      const mxArray *plhs[2]);

void c_compute_angular_velocity_roll(const mxArray *const prhs[2],
                                     const mxArray **plhs);

void c_compute_angular_velocity_yaw_(const mxArray *const prhs[2],
                                     const mxArray **plhs);

void compute_angular_velocity_C(real_T x[18], real_T z_n[9][3],
                                real_T omega[3]);

void compute_angular_velocity_C_api(const mxArray *const prhs[2],
                                    const mxArray **plhs);

void compute_angular_velocity_roll_C(real_T x[18], real_T z_n[9][3],
                                     real_T omega[3]);

void compute_angular_velocity_yaw_C(real_T x[18], real_T z_n[9][3],
                                    real_T omega[3]);

void compute_rotation_mat_C(real_T z_n[9][3], real_T theta[9], real_T C[3][3]);

void compute_rotation_mat_C_api(const mxArray *const prhs[2],
                                const mxArray **plhs);

void compute_rotation_mat_roll_C(real_T z_n[9][3], real_T theta[9],
                                 real_T C[3][3]);

void compute_rotation_mat_roll_C_api(const mxArray *const prhs[2],
                                     const mxArray **plhs);

void compute_rotation_mat_yaw_C(real_T z_n[9][3], real_T theta[9],
                                real_T C[3][3]);

void compute_rotation_mat_yaw_C_api(const mxArray *const prhs[2],
                                    const mxArray **plhs);

void libbitonestep_atexit(void);

void libbitonestep_initialize(void);

void libbitonestep_terminate(void);

void libbitonestep_xil_shutdown(void);

void libbitonestep_xil_terminate(void);

void rot2axis_C(real_T C[3][3], real_T v[3], real_T *phi);

void rot2axis_C_api(const mxArray *prhs, int32_T nlhs, const mxArray *plhs[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_libbitonestep_api.h
 *
 * [EOF]
 */
