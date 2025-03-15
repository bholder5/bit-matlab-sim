/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rot2axis_C.h
 *
 * Code generation for function 'rot2axis_C'
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
void rot2axis_C(const emlrtStack *sp, const real_T C[9], real_T v[3],
                real_T *phi);

/* End of code generation (rot2axis_C.h) */
