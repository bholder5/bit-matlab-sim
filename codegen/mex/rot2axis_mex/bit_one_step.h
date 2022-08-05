/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * bit_one_step.h
 *
 * Code generation for function 'bit_one_step'
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
void bit_one_step(const emlrtStack *sp, const real_T x0[21],
                  const real_T tau_applied[9], real_T y_true[21],
                  real_T phi_true[3], real_T w_k_true[3]);

/* End of code generation (bit_one_step.h) */
