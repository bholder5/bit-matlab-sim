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
  int32_T cond_ends_0_0 = 411;
  int32_T cond_starts_0_0 = 399;
  mex_InitInfAndNan();
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/bit_one_step.m", 0U, 3U, 6U, 1U,
                  0U, 0U, 0U, 1U, 0U, 1U, 1U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 0U, 0U, "bit_one_step", 0, -1, 2957);
  covrtFcnInit(&emlrtCoverageInstance, 0U, 1U, "bit_one_step_anonFcn1", 655, -1,
               850);
  covrtFcnInit(&emlrtCoverageInstance, 0U, 2U, "bit_one_step_anonFcn2", 1100,
               -1, 1192);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 5U, 1290, -1, 2942);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 2U, 538, -1, 1193);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 1U, 420, -1, 451);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 0U, 201, -1, 389);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 3U, 655, -1, 850);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 4U, 1100, -1, 1192);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 0U, 0U, 395, 411, -1, 459);
  /* Initialize MCDC Information */
  covrtMcdcInit(&emlrtCoverageInstance, 0U, 0U, 398, 411, 1, 0,
                &cond_starts_0_0, &cond_ends_0_0, 2, postfix_exprs_0_0);
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 0U, 0U, 1207, 1229, 2952);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 0U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/init_func.m", 1U, 1U, 3U, 0U,
                  0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 1U, 0U, "init_func", 0, -1, 3272);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 2U, 1944, -1, 3266);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 1U, 1764, -1, 1938);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 0U, 340, -1, 1747);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 1U, 0U, 1748, 1759, 1942);
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
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/bholder/bit-matlab-sim/Plant_functions/bit_propagator.m", 7U, 1U,
      8U, 3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 7U, 0U, "bit_propagator", 0, -1, 2033);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 7U, 1913, -1, 2024);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 5U, 1539, -1, 1697);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 6U, 1725, -1, 1887);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 4U, 1429, -1, 1480);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 3U, 1190, -1, 1367);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 2U, 464, -1, 1156);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 1U, 421, -1, 439);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 0U, 211, -1, 350);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 7U, 0U, 393, 412, -1, 447);
  covrtIfInit(&emlrtCoverageInstance, 7U, 1U, 1162, 1181, -1, 1907);
  covrtIfInit(&emlrtCoverageInstance, 7U, 2U, 1493, 1522, 1898, 1899);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  covrtWhileInit(&emlrtCoverageInstance, 7U, 0U, 1377, 1403, 1899);
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 7U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/Plant_functions/"
                  "compute_potential_energy_term.m",
                  8U, 1U, 14U, 3U, 0U, 0U, 0U, 7U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 8U, 0U, "compute_potential_energy_term",
               0, -1, 2671);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 13U, 2644, -1, 2667);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 12U, 2515, -1, 2607);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 11U, 2341, -1, 2368);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 10U, 2259, -1, 2291);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 9U, 1936, -1, 1951);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 8U, 1803, -1, 1837);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 7U, 1715, -1, 1742);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 6U, 1633, -1, 1665);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 5U, 1367, -1, 1382);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 4U, 1092, -1, 1132);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 3U, 1017, -1, 1036);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 2U, 853, -1, 897);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 1U, 560, -1, 712);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 0U, 382, -1, 527);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 8U, 0U, 1066, 1079, -1, 2635);
  covrtIfInit(&emlrtCoverageInstance, 8U, 1U, 1597, 1608, 1686, 1766);
  covrtIfInit(&emlrtCoverageInstance, 8U, 2U, 2223, 2234, 2312, 2392);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 8U, 0U, 537, 551, 720);
  covrtForInit(&emlrtCoverageInstance, 8U, 1U, 829, 843, 2671);
  covrtForInit(&emlrtCoverageInstance, 8U, 2U, 990, 1004, 1048);
  covrtForInit(&emlrtCoverageInstance, 8U, 3U, 1337, 1350, 1853);
  covrtForInit(&emlrtCoverageInstance, 8U, 4U, 1416, 1429, 1786);
  covrtForInit(&emlrtCoverageInstance, 8U, 5U, 1908, 1919, 2623);
  covrtForInit(&emlrtCoverageInstance, 8U, 6U, 1985, 1996, 2412);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 8U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/Miscellaneous/axis2rot.m", 9U,
                  1U, 4U, 1U, 0U, 0U, 0U, 2U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 9U, 0U, "axis2rot", 0, -1, 718);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 9U, 3U, 545, -1, 678);
  covrtBasicBlockInit(&emlrtCoverageInstance, 9U, 2U, 390, -1, 439);
  covrtBasicBlockInit(&emlrtCoverageInstance, 9U, 1U, 324, -1, 349);
  covrtBasicBlockInit(&emlrtCoverageInstance, 9U, 0U, 190, -1, 270);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 9U, 0U, 362, 373, 452, 694);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 9U, 0U, 280, 291, 714);
  covrtForInit(&emlrtCoverageInstance, 9U, 1U, 300, 311, 706);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 9U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/Plant_functions/RW_terms.m",
                  10U, 1U, 6U, 4U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 10U, 0U, "RW_terms", 0, -1, 677);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 10U, 5U, 509, -1, 562);
  covrtBasicBlockInit(&emlrtCoverageInstance, 10U, 4U, 483, -1, 495);
  covrtBasicBlockInit(&emlrtCoverageInstance, 10U, 3U, 408, -1, 420);
  covrtBasicBlockInit(&emlrtCoverageInstance, 10U, 2U, 334, -1, 357);
  covrtBasicBlockInit(&emlrtCoverageInstance, 10U, 1U, 249, -1, 323);
  covrtBasicBlockInit(&emlrtCoverageInstance, 10U, 0U, 216, -1, 232);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 10U, 0U, 359, 380, 429, 504);
  covrtIfInit(&emlrtCoverageInstance, 10U, 1U, 385, 399, -1, 428);
  covrtIfInit(&emlrtCoverageInstance, 10U, 2U, 429, 455, -1, 504);
  covrtIfInit(&emlrtCoverageInstance, 10U, 3U, 460, 474, -1, 503);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 10U, 0U, 233, 244, 331);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 10U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/mass_mat_func_gb.m", 11U, 4U,
                  4U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 11U, 0U, "mass_mat_func_gb", 0, -1,
               12082);
  covrtFcnInit(&emlrtCoverageInstance, 11U, 1U, "ft_1", 12083, -1, 28993);
  covrtFcnInit(&emlrtCoverageInstance, 11U, 2U, "ft_2", 28994, -1, 48383);
  covrtFcnInit(&emlrtCoverageInstance, 11U, 3U, "ft_3", 48384, -1, 62902);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 11U, 0U, 187, -1, 12078);
  covrtBasicBlockInit(&emlrtCoverageInstance, 11U, 1U, 12105, -1, 28989);
  covrtBasicBlockInit(&emlrtCoverageInstance, 11U, 2U, 29016, -1, 48379);
  covrtBasicBlockInit(&emlrtCoverageInstance, 11U, 3U, 48406, -1, 62898);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 11U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/bholder/bit-matlab-sim/flexible_model_data/flex_propogate.m", 12U,
      1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 12U, 0U, "flex_propogate", 0, -1, 577);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 12U, 0U, 168, -1, 573);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 12U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/compute_angular_velocity_C.m",
                  13U, 1U, 3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 13U, 0U, "compute_angular_velocity_C", 0,
               -1, 313);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 13U, 2U, 288, -1, 308);
  covrtBasicBlockInit(&emlrtCoverageInstance, 13U, 1U, 204, -1, 278);
  covrtBasicBlockInit(&emlrtCoverageInstance, 13U, 0U, 135, -1, 187);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 13U, 0U, 188, 199, 286);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 13U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/bholder/bit-matlab-sim/compute_angular_velocity_roll_C.m", 14U, 1U,
      3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 14U, 0U,
               "compute_angular_velocity_roll_C", 0, -1, 318);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 14U, 2U, 293, -1, 313);
  covrtBasicBlockInit(&emlrtCoverageInstance, 14U, 1U, 209, -1, 283);
  covrtBasicBlockInit(&emlrtCoverageInstance, 14U, 0U, 140, -1, 192);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 14U, 0U, 193, 204, 291);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 14U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/bholder/bit-matlab-sim/compute_angular_velocity_yaw_C.m", 15U, 1U,
      3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 15U, 0U,
               "compute_angular_velocity_yaw_C", 0, -1, 317);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 15U, 2U, 292, -1, 312);
  covrtBasicBlockInit(&emlrtCoverageInstance, 15U, 1U, 208, -1, 282);
  covrtBasicBlockInit(&emlrtCoverageInstance, 15U, 0U, 139, -1, 191);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 15U, 0U, 192, 203, 290);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 15U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/compute_rotation_mat_C.m", 16U,
                  1U, 3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 16U, 0U, "compute_rotation_mat_C", 0, -1,
               214);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 16U, 2U, 203, -1, 210);
  covrtBasicBlockInit(&emlrtCoverageInstance, 16U, 1U, 161, -1, 198);
  covrtBasicBlockInit(&emlrtCoverageInstance, 16U, 0U, 131, -1, 144);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 16U, 0U, 145, 156, 202);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 16U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/compute_rotation_mat_roll_C.m",
                  17U, 1U, 3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 17U, 0U, "compute_rotation_mat_roll_C",
               0, -1, 219);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 17U, 2U, 208, -1, 215);
  covrtBasicBlockInit(&emlrtCoverageInstance, 17U, 1U, 166, -1, 203);
  covrtBasicBlockInit(&emlrtCoverageInstance, 17U, 0U, 136, -1, 149);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 17U, 0U, 150, 161, 207);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 17U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/compute_rotation_mat_yaw_C.m",
                  18U, 1U, 3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 18U, 0U, "compute_rotation_mat_yaw_C", 0,
               -1, 218);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 18U, 2U, 207, -1, 214);
  covrtBasicBlockInit(&emlrtCoverageInstance, 18U, 1U, 165, -1, 202);
  covrtBasicBlockInit(&emlrtCoverageInstance, 18U, 0U, 135, -1, 148);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 18U, 0U, 149, 160, 206);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 18U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/bholder/bit-matlab-sim/rot2axis_C.m", 19U, 1U, 1U, 0U,
                  0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 19U, 0U, "rot2axis_C", 0, -1, 204);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 19U, 0U, 37, -1, 200);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 19U);
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
