/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * axis2rot.h
 *
 * Code generation for function 'axis2rot'
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
void axis2rot(const emlrtStack *sp, const real_T v[3], real32_T phi,
              real_T rot[9]);

void b_axis2rot(const emlrtStack *sp, const real_T v[3], real_T phi,
                real_T rot[9]);

/* End of code generation (axis2rot.h) */
