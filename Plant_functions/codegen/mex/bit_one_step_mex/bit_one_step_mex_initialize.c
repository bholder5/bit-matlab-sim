/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * bit_one_step_mex_initialize.c
 *
 * Code generation for function 'bit_one_step_mex_initialize'
 *
 */

/* Include files */
#include "bit_one_step_mex_initialize.h"
#include "_coder_bit_one_step_mex_mex.h"
#include "bit_one_step_mex_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void bit_one_step_mex_once(void);

/* Function Definitions */
static void bit_one_step_mex_once(void)
{
  int32_T postfix_exprs_0_0[2] = {0, -1};
  int32_T cond_ends_0_0 = 678;
  int32_T cond_starts_0_0 = 666;
  mex_InitInfAndNan();
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/bit_one_step.m", 0U, 3U, 7U, 2U,
                  0U, 0U, 0U, 1U, 0U, 1U, 1U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 0U, 0U, "bit_one_step", 0, -1, 3233);
  covrtFcnInit(&emlrtCoverageInstance, 0U, 1U, "bit_one_step_anonFcn1", 922, -1,
               1126);
  covrtFcnInit(&emlrtCoverageInstance, 0U, 2U, "bit_one_step_anonFcn2", 1376,
               -1, 1468);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 6U, 1566, -1, 3218);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 3U, 805, -1, 1469);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 2U, 687, -1, 718);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 1U, 447, -1, 643);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 0U, 229, -1, 428);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 4U, 922, -1, 1126);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 5U, 1376, -1, 1468);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 0U, 0U, 210, 220, 433, 651);
  covrtIfInit(&emlrtCoverageInstance, 0U, 1U, 662, 678, -1, 726);
  /* Initialize MCDC Information */
  covrtMcdcInit(&emlrtCoverageInstance, 0U, 0U, 665, 678, 1, 0,
                &cond_starts_0_0, &cond_ends_0_0, 2, postfix_exprs_0_0);
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 0U, 0U, 1483, 1505, 3228);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 0U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/init_func_sb.m", 1U, 1U, 3U, 0U,
                  0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 1U, 0U, "init_func_sb", 0, -1, 4889);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 2U, 3441, -1, 4883);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 1U, 3261, -1, 3435);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 0U, 343, -1, 3244);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 1U, 0U, 3245, 3256, 3439);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 1U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/Miscellaneous/xmat.m", 2U, 1U,
                  1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 2U, 0U, "xmat", 0, -1, 149);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 2U, 0U, 29, -1, 145);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 2U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/flexible_model_data/a_f_func.m",
                  3U, 1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 3U, 0U, "a_f_func", 0, -1, 45310);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 3U, 0U, 167, -1, 45306);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 3U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/flexible_model_data/b_f_func.m",
                  4U, 1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 4U, 0U, "b_f_func", 0, -1, 7565);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 0U, 167, -1, 7561);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 4U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/bholder/bit-matlab-sim/flexible_model_data/a_mf_func.m", 5U, 1U,
      1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 5U, 0U, "a_mf_func", 0, -1, 46316);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 5U, 0U, 172, -1, 46312);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 5U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/bholder/bit-matlab-sim/flexible_model_data/b_mf_func.m", 6U, 1U,
      1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 6U, 0U, "b_mf_func", 0, -1, 7556);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 0U, 172, -1, 7552);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 6U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/init_func.m", 7U, 1U, 3U, 0U,
                  0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 7U, 0U, "init_func", 0, -1, 3282);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 2U, 1953, -1, 3276);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 1U, 1773, -1, 1947);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 0U, 340, -1, 1756);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 7U, 0U, 1757, 1768, 1951);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 7U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/bholder/bit-matlab-sim/Plant_functions/bit_propagator.m", 8U, 1U,
      13U, 5U, 0U, 0U, 0U, 0U, 1U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 8U, 0U, "bit_propagator", 0, -1, 2270);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 12U, 2150, -1, 2261);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 10U, 1776, -1, 1934);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 11U, 1962, -1, 2124);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 9U, 1666, -1, 1717);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 8U, 1427, -1, 1604);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 7U, 1296, -1, 1393);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 6U, 1186, -1, 1214);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 5U, 1139, -1, 1167);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 4U, 665, -1, 1047);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 3U, 619, -1, 650);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 2U, 569, -1, 600);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 1U, 430, -1, 448);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 0U, 220, -1, 359);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 8U, 0U, 402, 421, -1, 456);
  covrtIfInit(&emlrtCoverageInstance, 8U, 1U, 550, 560, 605, 658);
  covrtIfInit(&emlrtCoverageInstance, 8U, 2U, 1120, 1130, 1172, 1222);
  covrtIfInit(&emlrtCoverageInstance, 8U, 3U, 1399, 1418, -1, 2144);
  covrtIfInit(&emlrtCoverageInstance, 8U, 4U, 1730, 1759, 2135, 2136);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  covrtWhileInit(&emlrtCoverageInstance, 8U, 0U, 1614, 1640, 2136);
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 8U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/bholder/bit-matlab-sim/Plant_functions/poten_mat_func_sb.m", 9U,
      1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 9U, 0U, "poten_mat_func_sb", 0, -1, 802);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 9U, 0U, 190, -1, 798);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 9U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/bholder/bit-matlab-sim/Plant_functions/poten_mat_func_gb.m", 10U,
      1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 10U, 0U, "poten_mat_func_gb", 0, -1,
               667);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 10U, 0U, 190, -1, 663);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 10U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/Plant_functions/RW_terms.m",
                  11U, 1U, 6U, 4U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 11U, 0U, "RW_terms", 0, -1, 677);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 11U, 5U, 509, -1, 562);
  covrtBasicBlockInit(&emlrtCoverageInstance, 11U, 4U, 483, -1, 495);
  covrtBasicBlockInit(&emlrtCoverageInstance, 11U, 3U, 408, -1, 420);
  covrtBasicBlockInit(&emlrtCoverageInstance, 11U, 2U, 334, -1, 357);
  covrtBasicBlockInit(&emlrtCoverageInstance, 11U, 1U, 249, -1, 323);
  covrtBasicBlockInit(&emlrtCoverageInstance, 11U, 0U, 216, -1, 232);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 11U, 0U, 359, 380, 429, 504);
  covrtIfInit(&emlrtCoverageInstance, 11U, 1U, 385, 399, -1, 428);
  covrtIfInit(&emlrtCoverageInstance, 11U, 2U, 429, 455, -1, 504);
  covrtIfInit(&emlrtCoverageInstance, 11U, 3U, 460, 474, -1, 503);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 11U, 0U, 233, 244, 331);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 11U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/Miscellaneous/axis2rot.m", 12U,
                  1U, 4U, 1U, 0U, 0U, 0U, 2U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 12U, 0U, "axis2rot", 0, -1, 718);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 12U, 3U, 545, -1, 678);
  covrtBasicBlockInit(&emlrtCoverageInstance, 12U, 2U, 390, -1, 439);
  covrtBasicBlockInit(&emlrtCoverageInstance, 12U, 1U, 324, -1, 349);
  covrtBasicBlockInit(&emlrtCoverageInstance, 12U, 0U, 190, -1, 270);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 12U, 0U, 362, 373, 452, 694);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 12U, 0U, 280, 291, 714);
  covrtForInit(&emlrtCoverageInstance, 12U, 1U, 300, 311, 706);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 12U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/mass_mat_func_sb.m", 13U, 4U,
                  4U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 13U, 0U, "mass_mat_func_sb", 0, -1,
               12250);
  covrtFcnInit(&emlrtCoverageInstance, 13U, 1U, "ft_1", 12251, -1, 29533);
  covrtFcnInit(&emlrtCoverageInstance, 13U, 2U, "ft_2", 29534, -1, 49239);
  covrtFcnInit(&emlrtCoverageInstance, 13U, 3U, "ft_3", 49240, -1, 64797);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 13U, 0U, 187, -1, 12246);
  covrtBasicBlockInit(&emlrtCoverageInstance, 13U, 1U, 12273, -1, 29529);
  covrtBasicBlockInit(&emlrtCoverageInstance, 13U, 2U, 29556, -1, 49235);
  covrtBasicBlockInit(&emlrtCoverageInstance, 13U, 3U, 49262, -1, 64793);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 13U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/mass_mat_func_gb.m", 14U, 4U,
                  4U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 14U, 0U, "mass_mat_func_gb", 0, -1,
               12110);
  covrtFcnInit(&emlrtCoverageInstance, 14U, 1U, "ft_1", 12111, -1, 29226);
  covrtFcnInit(&emlrtCoverageInstance, 14U, 2U, "ft_2", 29227, -1, 48722);
  covrtFcnInit(&emlrtCoverageInstance, 14U, 3U, "ft_3", 48723, -1, 63358);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 14U, 0U, 187, -1, 12106);
  covrtBasicBlockInit(&emlrtCoverageInstance, 14U, 1U, 12133, -1, 29222);
  covrtBasicBlockInit(&emlrtCoverageInstance, 14U, 2U, 29249, -1, 48718);
  covrtBasicBlockInit(&emlrtCoverageInstance, 14U, 3U, 48745, -1, 63354);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 14U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/bholder/bit-matlab-sim/flexible_model_data/flex_propogate.m", 15U,
      1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 15U, 0U, "flex_propogate", 0, -1, 577);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 15U, 0U, 168, -1, 573);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 15U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/compute_angular_velocity_C.m",
                  16U, 1U, 3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 16U, 0U, "compute_angular_velocity_C", 0,
               -1, 313);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 16U, 2U, 288, -1, 308);
  covrtBasicBlockInit(&emlrtCoverageInstance, 16U, 1U, 204, -1, 278);
  covrtBasicBlockInit(&emlrtCoverageInstance, 16U, 0U, 135, -1, 187);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 16U, 0U, 188, 199, 286);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 16U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/bholder/bit-matlab-sim/compute_angular_velocity_roll_C.m", 17U, 1U,
      3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 17U, 0U,
               "compute_angular_velocity_roll_C", 0, -1, 318);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 17U, 2U, 293, -1, 313);
  covrtBasicBlockInit(&emlrtCoverageInstance, 17U, 1U, 209, -1, 283);
  covrtBasicBlockInit(&emlrtCoverageInstance, 17U, 0U, 140, -1, 192);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 17U, 0U, 193, 204, 291);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 17U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/bholder/bit-matlab-sim/compute_angular_velocity_yaw_C.m", 18U, 1U,
      3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 18U, 0U,
               "compute_angular_velocity_yaw_C", 0, -1, 317);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 18U, 2U, 292, -1, 312);
  covrtBasicBlockInit(&emlrtCoverageInstance, 18U, 1U, 208, -1, 282);
  covrtBasicBlockInit(&emlrtCoverageInstance, 18U, 0U, 139, -1, 191);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 18U, 0U, 192, 203, 290);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 18U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/compute_rotation_mat_C.m", 19U,
                  1U, 3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 19U, 0U, "compute_rotation_mat_C", 0, -1,
               214);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 19U, 2U, 203, -1, 210);
  covrtBasicBlockInit(&emlrtCoverageInstance, 19U, 1U, 161, -1, 198);
  covrtBasicBlockInit(&emlrtCoverageInstance, 19U, 0U, 131, -1, 144);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 19U, 0U, 145, 156, 202);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 19U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/compute_rotation_mat_roll_C.m",
                  20U, 1U, 3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 20U, 0U, "compute_rotation_mat_roll_C",
               0, -1, 219);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 20U, 2U, 208, -1, 215);
  covrtBasicBlockInit(&emlrtCoverageInstance, 20U, 1U, 166, -1, 203);
  covrtBasicBlockInit(&emlrtCoverageInstance, 20U, 0U, 136, -1, 149);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 20U, 0U, 150, 161, 207);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 20U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/compute_rotation_mat_yaw_C.m",
                  21U, 1U, 3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 21U, 0U, "compute_rotation_mat_yaw_C", 0,
               -1, 218);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 21U, 2U, 207, -1, 214);
  covrtBasicBlockInit(&emlrtCoverageInstance, 21U, 1U, 165, -1, 202);
  covrtBasicBlockInit(&emlrtCoverageInstance, 21U, 0U, 135, -1, 148);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 21U, 0U, 149, 160, 206);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 21U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/rot2axis_C.m", 22U, 1U, 1U, 0U,
                  0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 22U, 0U, "rot2axis_C", 0, -1, 204);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 22U, 0U, 37, -1, 200);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 22U);
}

void bit_one_step_mex_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    bit_one_step_mex_once();
  }
}

/* End of code generation (bit_one_step_mex_initialize.c) */
