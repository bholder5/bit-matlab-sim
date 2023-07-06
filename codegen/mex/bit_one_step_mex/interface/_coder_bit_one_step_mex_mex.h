/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_bit_one_step_mex_mex.h
 *
 * Code generation for function '_coder_bit_one_step_mex_mex'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "covrt.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void bit_one_step_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs,
                              const mxArray *prhs[12]);

void compute_angular_velocity_C_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                            int32_T nrhs,
                                            const mxArray *prhs[2]);

void compute_angular_velocity_roll_C_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                                 int32_T nrhs,
                                                 const mxArray *prhs[2]);

void compute_angular_velocity_yaw_C_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                                int32_T nrhs,
                                                const mxArray *prhs[2]);

void compute_rotation_mat_C_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                        int32_T nrhs, const mxArray *prhs[2]);

void compute_rotation_mat_roll_C_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                             int32_T nrhs,
                                             const mxArray *prhs[2]);

void compute_rotation_mat_yaw_C_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                            int32_T nrhs,
                                            const mxArray *prhs[2]);

MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void rot2axis_C_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs,
                            const mxArray *prhs[1]);

/* End of code generation (_coder_bit_one_step_mex_mex.h) */
