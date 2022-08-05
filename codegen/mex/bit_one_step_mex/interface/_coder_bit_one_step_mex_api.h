/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_bit_one_step_mex_api.h
 *
 * Code generation for function '_coder_bit_one_step_mex_api'
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
void bit_one_step_api(const mxArray *const prhs[9], const mxArray **plhs);

void compute_angular_velocity_C_api(const mxArray *const prhs[2],
                                    const mxArray **plhs);

void compute_rotation_mat_C_api(const mxArray *const prhs[2],
                                const mxArray **plhs);

void rot2axis_C_api(const mxArray *prhs, int32_T nlhs, const mxArray *plhs[2]);

/* End of code generation (_coder_bit_one_step_mex_api.h) */
