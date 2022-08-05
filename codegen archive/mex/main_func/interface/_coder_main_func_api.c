/*
 * Prerelease License - for engineering feedback and testing purposes
 * only. Not for sale.
 *
 * _coder_main_func_api.c
 *
 * Code generation for function '_coder_main_func_api'
 *
 */

/* Include files */
#include "_coder_main_func_api.h"
#include "main_func.h"
#include "main_func_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void main_func_api(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  st.tls = emlrtRootTLSGlobal;
  /* Invoke the target function */
  main_func(&st);
}

/* End of code generation (_coder_main_func_api.c) */
