/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_rot2axis_mex_api.c
 *
 * Code generation for function '_coder_rot2axis_mex_api'
 *
 */

/* Include files */
#include "_coder_rot2axis_mex_api.h"
#include "bit_one_step.h"
#include "compute_rotation_mat.h"
#include "rot2axis.h"
#include "rot2axis_mex_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[21];

static const mxArray *b_emlrt_marshallOut(const real_T u[3]);

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *tau_applied,
                                   const char_T *identifier))[9];

static const mxArray *c_emlrt_marshallOut(const real_T u[9]);

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9];

static const mxArray *d_emlrt_marshallOut(const real_T u);

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *z_n,
                                   const char_T *identifier))[27];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *x0,
                                 const char_T *identifier))[21];

static const mxArray *emlrt_marshallOut(const real_T u[21]);

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[27];

static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *C,
                                   const char_T *identifier))[9];

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9];

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[21];

static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[9];

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[27];

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[9];

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[21]
{
  real_T(*y)[21];
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const real_T u[3])
{
  static const int32_T i = 0;
  static const int32_T i1 = 3;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *tau_applied,
                                   const char_T *identifier))[9]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[9];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(tau_applied), &thisId);
  emlrtDestroyArray(&tau_applied);
  return y;
}

static const mxArray *c_emlrt_marshallOut(const real_T u[9])
{
  static const int32_T iv[2] = {0, 0};
  static const int32_T iv1[2] = {3, 3};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9]
{
  real_T(*y)[9];
  y = j_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *d_emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *z_n,
                                   const char_T *identifier))[27]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[27];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(z_n), &thisId);
  emlrtDestroyArray(&z_n);
  return y;
}

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *x0,
                                 const char_T *identifier))[21]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[21];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(x0), &thisId);
  emlrtDestroyArray(&x0);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[21])
{
  static const int32_T i = 0;
  static const int32_T i1 = 21;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[27]
{
  real_T(*y)[27];
  y = k_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *C,
                                   const char_T *identifier))[9]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[9];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(C), &thisId);
  emlrtDestroyArray(&C);
  return y;
}

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9]
{
  real_T(*y)[9];
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[21]
{
  static const int32_T dims = 21;
  real_T(*ret)[21];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  ret = (real_T(*)[21])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[9]
{
  static const int32_T dims = 9;
  real_T(*ret)[9];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  ret = (real_T(*)[9])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[27]
{
  static const int32_T dims[2] = {3, 9};
  real_T(*ret)[27];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  ret = (real_T(*)[27])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[9]
{
  static const int32_T dims[2] = {3, 3};
  real_T(*ret)[9];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  ret = (real_T(*)[9])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void bit_one_step_api(const mxArray *const prhs[2], int32_T nlhs,
                      const mxArray *plhs[3])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*x0)[21];
  real_T(*y_true)[21];
  real_T(*tau_applied)[9];
  real_T(*phi_true)[3];
  real_T(*w_k_true)[3];
  st.tls = emlrtRootTLSGlobal;
  y_true = (real_T(*)[21])mxMalloc(sizeof(real_T[21]));
  phi_true = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  w_k_true = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  /* Marshall function inputs */
  x0 = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "x0");
  tau_applied = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "tau_applied");
  /* Invoke the target function */
  bit_one_step(&st, *x0, *tau_applied, *y_true, *phi_true, *w_k_true);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*y_true);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*phi_true);
  }
  if (nlhs > 2) {
    plhs[2] = b_emlrt_marshallOut(*w_k_true);
  }
}

void compute_rotation_mat_api(const mxArray *const prhs[2],
                              const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*z_n)[27];
  real_T(*C)[9];
  real_T(*theta)[9];
  st.tls = emlrtRootTLSGlobal;
  C = (real_T(*)[9])mxMalloc(sizeof(real_T[9]));
  /* Marshall function inputs */
  z_n = e_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "z_n");
  theta = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "theta");
  /* Invoke the target function */
  compute_rotation_mat(&st, *z_n, *theta, *C);
  /* Marshall function outputs */
  *plhs = c_emlrt_marshallOut(*C);
}

void rot2axis_api(const mxArray *prhs, int32_T nlhs, const mxArray *plhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*C)[9];
  real_T(*v)[3];
  real_T phi;
  st.tls = emlrtRootTLSGlobal;
  v = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  /* Marshall function inputs */
  C = g_emlrt_marshallIn(&st, emlrtAlias(prhs), "C");
  /* Invoke the target function */
  rot2axis(&st, *C, *v, &phi);
  /* Marshall function outputs */
  plhs[0] = b_emlrt_marshallOut(*v);
  if (nlhs > 1) {
    plhs[1] = d_emlrt_marshallOut(phi);
  }
}

/* End of code generation (_coder_rot2axis_mex_api.c) */
