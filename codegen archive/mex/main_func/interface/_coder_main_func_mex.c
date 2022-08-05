/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 *
 * _coder_main_func_mex.c
 *
 * Code generation for function '_coder_main_func_mex'
 *
 */

/* Include files */
#include "_coder_main_func_mex.h"
#include "_coder_main_func_api.h"
#include "main_func_data.h"
#include "main_func_initialize.h"
#include "main_func_terminate.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void main_func_mexFunction(int32_T nlhs, int32_T nrhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 0, 4,
                        9, "main_func");
  }
  if (nlhs > 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 9,
                        "main_func");
  }
  /* Call the function. */
  main_func_api();
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  (void)plhs;
  (void)prhs;
  mexAtExit(&main_func_atexit);
  /* Module initialization. */
  main_func_initialize();
  /* Dispatch the entry-point. */
  main_func_mexFunction(nlhs, nrhs);
  /* Module termination. */
  main_func_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2021a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_main_func_mex.c) */
