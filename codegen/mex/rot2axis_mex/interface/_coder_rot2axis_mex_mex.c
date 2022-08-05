/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_rot2axis_mex_mex.c
 *
 * Code generation for function '_coder_rot2axis_mex_mex'
 *
 */

/* Include files */
#include "_coder_rot2axis_mex_mex.h"
#include "_coder_rot2axis_mex_api.h"
#include "rot2axis_mex_data.h"
#include "rot2axis_mex_initialize.h"
#include "rot2axis_mex_terminate.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void bit_one_step_mexFunction(int32_T nlhs, mxArray *plhs[3], int32_T nrhs,
                              const mxArray *prhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[3];
  int32_T nOutputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4,
                        12, "bit_one_step");
  }
  if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 12,
                        "bit_one_step");
  }
  /* Call the function. */
  bit_one_step_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    nOutputs = 1;
  } else {
    nOutputs = nlhs;
  }
  emlrtReturnArrays(nOutputs, &plhs[0], &outputs[0]);
}

void compute_rotation_mat_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                      int32_T nrhs, const mxArray *prhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4,
                        20, "compute_rotation_mat");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 20,
                        "compute_rotation_mat");
  }
  /* Call the function. */
  compute_rotation_mat_api(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  static const char_T *emlrtEntryPoints[3] = {
      "bit_one_step", "compute_rotation_mat", "rot2axis"};
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexAtExit(&rot2axis_mex_atexit);
  /* Module initialization. */
  rot2axis_mex_initialize();
  st.tls = emlrtRootTLSGlobal;
  /* Dispatch the entry-point. */
  switch (emlrtGetEntryPointIndexR2016a(
      &st, nrhs, &prhs[0], (const char_T **)&emlrtEntryPoints[0], 3)) {
  case 0:
    bit_one_step_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  case 1:
    compute_rotation_mat_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  case 2:
    rot2axis_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  }
  /* Module termination. */
  rot2axis_mex_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2021a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL);
  return emlrtRootTLSGlobal;
}

void rot2axis_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs,
                          const mxArray *prhs[1])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[2];
  int32_T nOutputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        8, "rot2axis");
  }
  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 8,
                        "rot2axis");
  }
  /* Call the function. */
  rot2axis_api(prhs[0], nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    nOutputs = 1;
  } else {
    nOutputs = nlhs;
  }
  emlrtReturnArrays(nOutputs, &plhs[0], &outputs[0]);
}

/* End of code generation (_coder_rot2axis_mex_mex.c) */
