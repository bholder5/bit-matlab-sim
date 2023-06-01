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
                  real_T tau_applied[9], const real_T unlock[9], real_T w_piv,
                  boolean_T piv_flag, real_T dt, uint16_T num_steps,
                  real_T tau_max_piv, real_T thet_pit_nom,
                  const real_T x_flex0[104], const real_T tau_flex[5],
                  real_T y_true[21], real_T y_flex[104]);

/* End of code generation (bit_one_step.h) */
