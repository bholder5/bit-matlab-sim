/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_libbitonestep_api.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 14-Mar-2024 11:31:39
 */

/* Include Files */
#include "_coder_libbitonestep_api.h"
#include "_coder_libbitonestep_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131642U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "libbitonestep",                                      /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

/* Function Declarations */
static real32_T (*ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                      const emlrtMsgIdentifier *msgId))[9];

static real32_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[21];

static const mxArray *b_emlrt_marshallOut(const real32_T u[104]);

static real32_T bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static real32_T (*c_emlrt_marshallIn(const emlrtStack *sp,
                                     const mxArray *tau_applied,
                                     const char_T *identifier))[9];

static const mxArray *c_emlrt_marshallOut(const real_T u[3]);

static boolean_T cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                     const emlrtMsgIdentifier *msgId);

static real32_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[9];

static const mxArray *d_emlrt_marshallOut(real_T u[3][3]);

static uint16_T db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static real32_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *w_piv,
                                   const char_T *identifier);

static const mxArray *e_emlrt_marshallOut(const real_T u);

static real32_T (*eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                      const emlrtMsgIdentifier *msgId))[104];

static real32_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *x0,
                                   const char_T *identifier))[21];

static const mxArray *emlrt_marshallOut(const real32_T u[21]);

static real32_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static real32_T (*fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                      const emlrtMsgIdentifier *msgId))[5];

static boolean_T g_emlrt_marshallIn(const emlrtStack *sp,
                                    const mxArray *piv_flag,
                                    const char_T *identifier);

static real_T (*gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[18];

static boolean_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static real_T (*hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[9][3];

static uint16_T i_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *num_steps,
                                   const char_T *identifier);

static real_T (*ib_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[9];

static uint16_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static real_T (*jb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[3][3];

static real32_T (*k_emlrt_marshallIn(const emlrtStack *sp,
                                     const mxArray *x_flex0,
                                     const char_T *identifier))[104];

static real32_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[104];

static real32_T (*o_emlrt_marshallIn(const emlrtStack *sp,
                                     const mxArray *tau_flex,
                                     const char_T *identifier))[5];

static real32_T (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[5];

static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x,
                                   const char_T *identifier))[18];

static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[18];

static real_T (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *z_n,
                                   const char_T *identifier))[9][3];

static real_T (*t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9][3];

static real_T (*u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *theta,
                                   const char_T *identifier))[9];

static real_T (*v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9];

static real_T (*w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *C,
                                   const char_T *identifier))[3][3];

static real_T (*x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3][3];

static real32_T (*y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                     const emlrtMsgIdentifier *msgId))[21];

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real32_T (*)[9]
 */
static real32_T (*ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                      const emlrtMsgIdentifier *msgId))[9]
{
  static const int32_T dims = 9;
  int32_T b_i;
  real32_T(*ret)[9];
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "single", false, 1U,
                            (const void *)(&dims), &b, &b_i);
  ret = (real32_T(*)[9])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real32_T (*)[21]
 */
static real32_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[21]
{
  real32_T(*y)[21];
  y = y_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const real32_T u[104]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const real32_T u[104])
{
  static const int32_T b_i = 0;
  static const int32_T i1 = 104;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)(&b_i), (int32_T)mxSINGLE_CLASS,
                              (int32_T)mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)(&u[0]));
  (void)emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real32_T
 */
static real32_T bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real32_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "single", false, 0U,
                          (const void *)(&dims));
  ret = *((real32_T *)emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *tau_applied
 *                const char_T *identifier
 * Return Type  : real32_T (*)[9]
 */
static real32_T (*c_emlrt_marshallIn(const emlrtStack *sp,
                                     const mxArray *tau_applied,
                                     const char_T *identifier))[9]
{
  emlrtMsgIdentifier thisId;
  real32_T(*y)[9];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(tau_applied), &thisId);
  emlrtDestroyArray(&tau_applied);
  return y;
}

/*
 * Arguments    : const real_T u[3]
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrt_marshallOut(const real_T u[3])
{
  static const int32_T b_i = 0;
  static const int32_T i1 = 3;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)(&b_i), (int32_T)mxDOUBLE_CLASS,
                              (int32_T)mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)(&u[0]));
  (void)emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : boolean_T
 */
static boolean_T cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                     const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  boolean_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "logical", false, 0U,
                          (const void *)(&dims));
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real32_T (*)[9]
 */
static real32_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[9]
{
  real32_T(*y)[9];
  y = ab_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : real_T u[3][3]
 * Return Type  : const mxArray *
 */
static const mxArray *d_emlrt_marshallOut(real_T u[3][3])
{
  static const int32_T iv[2] = {0, 0};
  static const int32_T iv1[2] = {3, 3};
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)(&iv[0]),
                              (int32_T)mxDOUBLE_CLASS, (int32_T)mxREAL);
  emlrtMxSetData((mxArray *)m, &u[0]);
  (void)emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : uint16_T
 */
static uint16_T db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  uint16_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "uint16", false, 0U,
                          (const void *)(&dims));
  ret = *((uint16_T *)emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *w_piv
 *                const char_T *identifier
 * Return Type  : real32_T
 */
static real32_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *w_piv,
                                   const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real32_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(w_piv), &thisId);
  emlrtDestroyArray(&w_piv);
  return y;
}

/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
static const mxArray *e_emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real32_T (*)[104]
 */
static real32_T (*eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                      const emlrtMsgIdentifier *msgId))[104]
{
  static const int32_T dims = 104;
  int32_T b_i;
  real32_T(*ret)[104];
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "single", false, 1U,
                            (const void *)(&dims), &b, &b_i);
  ret = (real32_T(*)[104])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *x0
 *                const char_T *identifier
 * Return Type  : real32_T (*)[21]
 */
static real32_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *x0,
                                   const char_T *identifier))[21]
{
  emlrtMsgIdentifier thisId;
  real32_T(*y)[21];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(x0), &thisId);
  emlrtDestroyArray(&x0);
  return y;
}

/*
 * Arguments    : const real32_T u[21]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real32_T u[21])
{
  static const int32_T b_i = 0;
  static const int32_T i1 = 21;
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)(&b_i), (int32_T)mxSINGLE_CLASS,
                              (int32_T)mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)(&u[0]));
  (void)emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real32_T
 */
static real32_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  real32_T y;
  y = bb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real32_T (*)[5]
 */
static real32_T (*fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                      const emlrtMsgIdentifier *msgId))[5]
{
  static const int32_T dims = 5;
  int32_T b_i;
  real32_T(*ret)[5];
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "single", false, 1U,
                            (const void *)(&dims), &b, &b_i);
  ret = (real32_T(*)[5])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *piv_flag
 *                const char_T *identifier
 * Return Type  : boolean_T
 */
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

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[18]
 */
static real_T (*gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[18]
{
  static const int32_T dims = 18;
  real_T(*ret)[18];
  int32_T b_i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)(&dims), &b, &b_i);
  ret = (real_T(*)[18])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : boolean_T
 */
static boolean_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = cb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[9][3]
 */
static real_T (*hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[9][3]
{
  static const int32_T dims[2] = {3, 9};
  real_T(*ret)[9][3];
  int32_T iv[2];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)(&dims[0]), &bv[0], &iv[0]);
  ret = (real_T(*)[9][3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *num_steps
 *                const char_T *identifier
 * Return Type  : uint16_T
 */
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

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[9]
 */
static real_T (*ib_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[9]
{
  static const int32_T dims = 9;
  real_T(*ret)[9];
  int32_T b_i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1U,
                            (const void *)(&dims), &b, &b_i);
  ret = (real_T(*)[9])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : uint16_T
 */
static uint16_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  uint16_T y;
  y = db_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[3][3]
 */
static real_T (*jb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId))[3][3]
{
  static const int32_T dims[2] = {3, 3};
  real_T(*ret)[3][3];
  int32_T iv[2];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)(&dims[0]), &bv[0], &iv[0]);
  ret = (real_T(*)[3][3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *x_flex0
 *                const char_T *identifier
 * Return Type  : real32_T (*)[104]
 */
static real32_T (*k_emlrt_marshallIn(const emlrtStack *sp,
                                     const mxArray *x_flex0,
                                     const char_T *identifier))[104]
{
  emlrtMsgIdentifier thisId;
  real32_T(*y)[104];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = m_emlrt_marshallIn(sp, emlrtAlias(x_flex0), &thisId);
  emlrtDestroyArray(&x_flex0);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real32_T (*)[104]
 */
static real32_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[104]
{
  real32_T(*y)[104];
  y = eb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *tau_flex
 *                const char_T *identifier
 * Return Type  : real32_T (*)[5]
 */
static real32_T (*o_emlrt_marshallIn(const emlrtStack *sp,
                                     const mxArray *tau_flex,
                                     const char_T *identifier))[5]
{
  emlrtMsgIdentifier thisId;
  real32_T(*y)[5];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = p_emlrt_marshallIn(sp, emlrtAlias(tau_flex), &thisId);
  emlrtDestroyArray(&tau_flex);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real32_T (*)[5]
 */
static real32_T (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                     const emlrtMsgIdentifier *parentId))[5]
{
  real32_T(*y)[5];
  y = fb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *x
 *                const char_T *identifier
 * Return Type  : real_T (*)[18]
 */
static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x,
                                   const char_T *identifier))[18]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[18];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = r_emlrt_marshallIn(sp, emlrtAlias(x), &thisId);
  emlrtDestroyArray(&x);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[18]
 */
static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[18]
{
  real_T(*y)[18];
  y = gb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *z_n
 *                const char_T *identifier
 * Return Type  : real_T (*)[9][3]
 */
static real_T (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *z_n,
                                   const char_T *identifier))[9][3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[9][3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = t_emlrt_marshallIn(sp, emlrtAlias(z_n), &thisId);
  emlrtDestroyArray(&z_n);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[9][3]
 */
static real_T (*t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9][3]
{
  real_T(*y)[9][3];
  y = hb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *theta
 *                const char_T *identifier
 * Return Type  : real_T (*)[9]
 */
static real_T (*u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *theta,
                                   const char_T *identifier))[9]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[9];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = v_emlrt_marshallIn(sp, emlrtAlias(theta), &thisId);
  emlrtDestroyArray(&theta);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[9]
 */
static real_T (*v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[9]
{
  real_T(*y)[9];
  y = ib_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *C
 *                const char_T *identifier
 * Return Type  : real_T (*)[3][3]
 */
static real_T (*w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *C,
                                   const char_T *identifier))[3][3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[3][3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = x_emlrt_marshallIn(sp, emlrtAlias(C), &thisId);
  emlrtDestroyArray(&C);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[3][3]
 */
static real_T (*x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3][3]
{
  real_T(*y)[3][3];
  y = jb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real32_T (*)[21]
 */
static real32_T (*y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                     const emlrtMsgIdentifier *msgId))[21]
{
  static const int32_T dims = 21;
  int32_T b_i;
  real32_T(*ret)[21];
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "single", false, 1U,
                            (const void *)(&dims), &b, &b_i);
  ret = (real32_T(*)[21])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const mxArray * const prhs[12]
 *                int32_T nlhs
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void bit_one_step_api(const mxArray *const prhs[12], int32_T nlhs,
                      const mxArray *plhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *prhs_copy_idx_1;
  real32_T(*x_flex0)[104];
  real32_T(*y_flex)[104];
  real32_T(*x0)[21];
  real32_T(*y_true)[21];
  real32_T(*tau_applied)[9];
  real32_T(*unlock)[9];
  real32_T(*tau_flex)[5];
  real32_T dt;
  real32_T tau_max_piv;
  real32_T thet_pit_nom;
  real32_T w_piv;
  uint16_T num_steps;
  boolean_T flexure_flag;
  boolean_T piv_flag;
  st.tls = emlrtRootTLSGlobal;
  y_true = (real32_T(*)[21])mxMalloc(sizeof(real32_T[21]));
  y_flex = (real32_T(*)[104])mxMalloc(sizeof(real32_T[104]));
  prhs_copy_idx_1 = emlrtProtectR2012b(prhs[1], 1, false, -1);
  /* Marshall function inputs */
  x0 = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "x0");
  tau_applied =
      c_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_1), "tau_applied");
  unlock = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "unlock");
  w_piv = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "w_piv");
  piv_flag = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "piv_flag");
  dt = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "dt");
  num_steps = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "num_steps");
  tau_max_piv = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "tau_max_piv");
  thet_pit_nom = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "thet_pit_nom");
  x_flex0 = k_emlrt_marshallIn(&st, emlrtAlias(prhs[9]), "x_flex0");
  tau_flex = o_emlrt_marshallIn(&st, emlrtAlias(prhs[10]), "tau_flex");
  flexure_flag = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[11]), "flexure_flag");
  /* Invoke the target function */
  bit_one_step(*x0, *tau_applied, *unlock, w_piv, piv_flag, dt, num_steps,
               tau_max_piv, thet_pit_nom, *x_flex0, *tau_flex, flexure_flag,
               *y_true, *y_flex);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*y_true);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(*y_flex);
  }
}

/*
 * Arguments    : const mxArray * const prhs[2]
 *                const mxArray **plhs
 * Return Type  : void
 */
void c_compute_angular_velocity_roll(const mxArray *const prhs[2],
                                     const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*z_n)[9][3];
  real_T(*x)[18];
  real_T(*omega)[3];
  st.tls = emlrtRootTLSGlobal;
  omega = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  /* Marshall function inputs */
  x = q_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "x");
  z_n = s_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "z_n");
  /* Invoke the target function */
  compute_angular_velocity_roll_C(*x, *z_n, *omega);
  /* Marshall function outputs */
  *plhs = c_emlrt_marshallOut(*omega);
}

/*
 * Arguments    : const mxArray * const prhs[2]
 *                const mxArray **plhs
 * Return Type  : void
 */
void c_compute_angular_velocity_yaw_(const mxArray *const prhs[2],
                                     const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*z_n)[9][3];
  real_T(*x)[18];
  real_T(*omega)[3];
  st.tls = emlrtRootTLSGlobal;
  omega = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  /* Marshall function inputs */
  x = q_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "x");
  z_n = s_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "z_n");
  /* Invoke the target function */
  compute_angular_velocity_yaw_C(*x, *z_n, *omega);
  /* Marshall function outputs */
  *plhs = c_emlrt_marshallOut(*omega);
}

/*
 * Arguments    : const mxArray * const prhs[2]
 *                const mxArray **plhs
 * Return Type  : void
 */
void compute_angular_velocity_C_api(const mxArray *const prhs[2],
                                    const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*z_n)[9][3];
  real_T(*x)[18];
  real_T(*omega)[3];
  st.tls = emlrtRootTLSGlobal;
  omega = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  /* Marshall function inputs */
  x = q_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "x");
  z_n = s_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "z_n");
  /* Invoke the target function */
  compute_angular_velocity_C(*x, *z_n, *omega);
  /* Marshall function outputs */
  *plhs = c_emlrt_marshallOut(*omega);
}

/*
 * Arguments    : const mxArray * const prhs[2]
 *                const mxArray **plhs
 * Return Type  : void
 */
void compute_rotation_mat_C_api(const mxArray *const prhs[2],
                                const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*z_n)[9][3];
  real_T(*C)[3][3];
  real_T(*theta)[9];
  st.tls = emlrtRootTLSGlobal;
  C = (real_T(*)[3][3])mxMalloc(sizeof(real_T[3][3]));
  /* Marshall function inputs */
  z_n = s_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "z_n");
  theta = u_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "theta");
  /* Invoke the target function */
  compute_rotation_mat_C(*z_n, *theta, *C);
  /* Marshall function outputs */
  *plhs = d_emlrt_marshallOut(*C);
}

/*
 * Arguments    : const mxArray * const prhs[2]
 *                const mxArray **plhs
 * Return Type  : void
 */
void compute_rotation_mat_roll_C_api(const mxArray *const prhs[2],
                                     const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*z_n)[9][3];
  real_T(*C)[3][3];
  real_T(*theta)[9];
  st.tls = emlrtRootTLSGlobal;
  C = (real_T(*)[3][3])mxMalloc(sizeof(real_T[3][3]));
  /* Marshall function inputs */
  z_n = s_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "z_n");
  theta = u_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "theta");
  /* Invoke the target function */
  compute_rotation_mat_roll_C(*z_n, *theta, *C);
  /* Marshall function outputs */
  *plhs = d_emlrt_marshallOut(*C);
}

/*
 * Arguments    : const mxArray * const prhs[2]
 *                const mxArray **plhs
 * Return Type  : void
 */
void compute_rotation_mat_yaw_C_api(const mxArray *const prhs[2],
                                    const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*z_n)[9][3];
  real_T(*C)[3][3];
  real_T(*theta)[9];
  st.tls = emlrtRootTLSGlobal;
  C = (real_T(*)[3][3])mxMalloc(sizeof(real_T[3][3]));
  /* Marshall function inputs */
  z_n = s_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "z_n");
  theta = u_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "theta");
  /* Invoke the target function */
  compute_rotation_mat_yaw_C(*z_n, *theta, *C);
  /* Marshall function outputs */
  *plhs = d_emlrt_marshallOut(*C);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void libbitonestep_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  (void)mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  libbitonestep_xil_terminate();
  libbitonestep_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void libbitonestep_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  (void)mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  (void)emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void libbitonestep_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : const mxArray *prhs
 *                int32_T nlhs
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void rot2axis_C_api(const mxArray *prhs, int32_T nlhs, const mxArray *plhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*C)[3][3];
  real_T(*v)[3];
  real_T phi;
  st.tls = emlrtRootTLSGlobal;
  v = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  /* Marshall function inputs */
  C = w_emlrt_marshallIn(&st, emlrtAlias(prhs), "C");
  /* Invoke the target function */
  rot2axis_C(*C, *v, &phi);
  /* Marshall function outputs */
  plhs[0] = c_emlrt_marshallOut(*v);
  if (nlhs > 1) {
    plhs[1] = e_emlrt_marshallOut(phi);
  }
}

/*
 * File trailer for _coder_libbitonestep_api.c
 *
 * [EOF]
 */
