/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * bit_propagator.h
 *
 * Code generation for function 'bit_propagator'
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
void bit_propagator(const emlrtStack *sp, const real_T X[21],
                    real_T tau_applied[9], real_T Xdot[21]);

/* End of code generation (bit_propagator.h) */
