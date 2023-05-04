/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_bit_one_step_mex_api.c
 *
 * Code generation for function '_coder_bit_one_step_mex_api'
 *
 */

/* Include files */
#include "_coder_bit_one_step_mex_api.h"
#include "bit_one_step.h"
#include "bit_one_step_mex_data.h"
#include "compute_angular_velocity_C.h"
#include "compute_rotation_mat_C.h"
#include "rot2axis_C.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static real_T (*ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[104];

static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[21];

static const mxArray *b_emlrt_marshallOut(const real_T u[104]);

static real_T (*bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[5];

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *tau_applied,
                                   const char_T *identifier))[9];

static const mxArray *c_emlrt_marshallOut(const real_T u[3]);

static real_T (*cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[18];

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9];

static const mxArray *d_emlrt_marshallOut(const real_T u[9]);

static real_T (*db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[27];

static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *w_piv,
                                 const char_T *identifier);

static const mxArray *e_emlrt_marshallOut(const real_T u);

static real_T (*eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[9];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *x0,
                                 const char_T *identifier))[21];

static const mxArray *emlrt_marshallOut(const real_T u[21]);

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static boolean_T g_emlrt_marshallIn(const emlrtStack *sp,
                                    const mxArray *piv_flag,
                                    const char_T *identifier);

static boolean_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static uint16_T i_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *num_steps,
                                   const char_T *identifier);

static uint16_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x_flex0,
                                   const char_T *identifier))[104];

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[104];

static real_T (*m_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *tau_flex,
                                   const char_T *identifier))[5];

static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[5];

static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x,
                                   const char_T *identifier))[18];

static real_T (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[18];

static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *z_n,
                                   const char_T *identifier))[27];

static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[27];

static real_T (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *C,
                                   const char_T *identifier))[9];

static real_T (*t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9];

static real_T (*u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[21];

static real_T (*v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[9];

static real_T w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static boolean_T x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static uint16_T y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId);

/* Function Definitions */
static real_T (*ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[104]
{
  static const int32_T dims = 104;
  real_T(*ret)[104];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[104])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[21]
{
  real_T(*y)[21];
  y = u_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const real_T u[104])
{
  static const int32_T i = 0;
  static const int32_T i1 = 104;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[5]
{
  static const int32_T dims = 5;
  real_T(*ret)[5];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[5])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
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

static const mxArray *c_emlrt_marshallOut(const real_T u[3])
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

static real_T (*cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[18]
{
  static const int32_T dims = 18;
  real_T(*ret)[18];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[18])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9]
{
  real_T(*y)[9];
  y = v_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *d_emlrt_marshallOut(const real_T u[9])
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

static real_T (*db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[27]
{
  static const int32_T dims[2] = {3, 9};
  real_T(*ret)[27];
  int32_T iv[2];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[27])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *w_piv,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(w_piv), &thisId);
  emlrtDestroyArray(&w_piv);
  return y;
}

static const mxArray *e_emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[9]
{
  static const int32_T dims[2] = {3, 3};
  real_T(*ret)[9];
  int32_T iv[2];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[9])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
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

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = w_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static boolean_T g_emlrt_marshallIn(const emlrtStack *sp,
                                    const mxArray *piv_flag,
                                    const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  boolean_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(piv_flag), &thisId);
  emlrtDestroyArray(&piv_flag);
  return y;
}

static boolean_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = x_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static uint16_T i_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *num_steps,
                                   const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  uint16_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = j_emlrt_marshallIn(sp, emlrtAlias(num_steps), &thisId);
  emlrtDestroyArray(&num_steps);
  return y;
}

static uint16_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  uint16_T y;
  y = y_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x_flex0,
                                   const char_T *identifier))[104]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[104];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = l_emlrt_marshallIn(sp, emlrtAlias(x_flex0), &thisId);
  emlrtDestroyArray(&x_flex0);
  return y;
}

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[104]
{
  real_T(*y)[104];
  y = ab_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*m_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *tau_flex,
                                   const char_T *identifier))[5]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[5];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = n_emlrt_marshallIn(sp, emlrtAlias(tau_flex), &thisId);
  emlrtDestroyArray(&tau_flex);
  return y;
}

static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[5]
{
  real_T(*y)[5];
  y = bb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x,
                                   const char_T *identifier))[18]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[18];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = p_emlrt_marshallIn(sp, emlrtAlias(x), &thisId);
  emlrtDestroyArray(&x);
  return y;
}

static real_T (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[18]
{
  real_T(*y)[18];
  y = cb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *z_n,
                                   const char_T *identifier))[27]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[27];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = r_emlrt_marshallIn(sp, emlrtAlias(z_n), &thisId);
  emlrtDestroyArray(&z_n);
  return y;
}

static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[27]
{
  real_T(*y)[27];
  y = db_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *C,
                                   const char_T *identifier))[9]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[9];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = t_emlrt_marshallIn(sp, emlrtAlias(C), &thisId);
  emlrtDestroyArray(&C);
  return y;
}

static real_T (*t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9]
{
  real_T(*y)[9];
  y = eb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[21]
{
  static const int32_T dims = 21;
  real_T(*ret)[21];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[21])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[9]
{
  static const int32_T dims = 9;
  real_T(*ret)[9];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[9])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static boolean_T x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  boolean_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "logical", false, 0U,
                          (const void *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

static uint16_T y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  uint16_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "uint16", false, 0U,
                          (const void *)&dims);
  ret = *(uint16_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void bit_one_step_api(const mxArray *const prhs[11], int32_T nlhs,
                      const mxArray *plhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*x_flex0)[104];
  real_T(*y_flex)[104];
  real_T(*x0)[21];
  real_T(*y_true)[21];
  real_T(*tau_applied)[9];
  real_T(*unlock)[9];
  real_T(*tau_flex)[5];
  real_T dt;
  real_T tau_max_piv;
  real_T thet_pit_nom;
  real_T w_piv;
  uint16_T num_steps;
  boolean_T piv_flag;
  st.tls = emlrtRootTLSGlobal;
  y_true = (real_T(*)[21])mxMalloc(sizeof(real_T[21]));
  y_flex = (real_T(*)[104])mxMalloc(sizeof(real_T[104]));
  /* Marshall function inputs */
  x0 = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "x0");
  tau_applied = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "tau_applied");
  unlock = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "unlock");
  w_piv = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "w_piv");
  piv_flag = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "piv_flag");
  dt = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "dt");
  num_steps = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "num_steps");
  tau_max_piv = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "tau_max_piv");
  thet_pit_nom = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "thet_pit_nom");
  x_flex0 = k_emlrt_marshallIn(&st, emlrtAlias(prhs[9]), "x_flex0");
  tau_flex = m_emlrt_marshallIn(&st, emlrtAlias(prhs[10]), "tau_flex");
  /* Invoke the target function */
  bit_one_step(&st, *x0, *tau_applied, *unlock, w_piv, piv_flag, dt, num_steps,
               tau_max_piv, thet_pit_nom, *x_flex0, *tau_flex, *y_true,
               *y_flex);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*y_true);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*y_flex);
  }
}

void compute_angular_velocity_C_api(const mxArray *const prhs[2],
                                    const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*z_n)[27];
  real_T(*x)[18];
  real_T(*omega)[3];
  st.tls = emlrtRootTLSGlobal;
  omega = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  /* Marshall function inputs */
  x = o_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "x");
  z_n = q_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "z_n");
  /* Invoke the target function */
  compute_angular_velocity_C(&st, *x, *z_n, *omega);
  /* Marshall function outputs */
  *plhs = c_emlrt_marshallOut(*omega);
}

void compute_rotation_mat_C_api(const mxArray *const prhs[2],
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
  z_n = q_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "z_n");
  theta = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "theta");
  /* Invoke the target function */
  compute_rotation_mat_C(&st, *z_n, *theta, *C);
  /* Marshall function outputs */
  *plhs = d_emlrt_marshallOut(*C);
}

void rot2axis_C_api(const mxArray *prhs, int32_T nlhs, const mxArray *plhs[2])
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
  C = s_emlrt_marshallIn(&st, emlrtAlias(prhs), "C");
  /* Invoke the target function */
  rot2axis_C(&st, *C, *v, &phi);
  /* Marshall function outputs */
  plhs[0] = c_emlrt_marshallOut(*v);
  if (nlhs > 1) {
    plhs[1] = e_emlrt_marshallOut(phi);
  }
}

/* End of code generation (_coder_bit_one_step_mex_api.c) */
