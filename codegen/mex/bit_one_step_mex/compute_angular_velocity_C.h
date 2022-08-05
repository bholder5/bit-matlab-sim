/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_angular_velocity_C.h
 *
 * Code generation for function 'compute_angular_velocity_C'
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
void compute_angular_velocity_C(const emlrtStack *sp, const real_T x[18],
                                const real_T z_n[27], real_T omega[3]);

/* End of code generation (compute_angular_velocity_C.h) */
