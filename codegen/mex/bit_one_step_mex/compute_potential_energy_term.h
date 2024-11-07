/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_potential_energy_term.h
 *
 * Code generation for function 'compute_potential_energy_term'
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
void compute_potential_energy_term(const emlrtStack *sp, const real_T theta[9],
                                   const real_T c_n[27], const real_T z_n[27],
                                   const real_T m_n[9], const real_T r_n1_n[27],
                                   const real_T g0[3], real_T Pot[9]);

/* End of code generation (compute_potential_energy_term.h) */
