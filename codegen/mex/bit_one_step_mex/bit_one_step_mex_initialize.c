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
  mex_InitInfAndNan();
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/brad/bit-matlab-sim/bit_one_step.m", 0U, 2U, 3U, 0U,
                  0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 0U, 0U, "bit_one_step", 0, -1, 1969);
  covrtFcnInit(&emlrtCoverageInstance, 0U, 1U, "bit_one_step_anonFcn1", 492, -1,
               687);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 2U, 784, -1, 1369);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 0U, 160, -1, 688);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 1U, 492, -1, 687);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 0U, 0U, 701, 723, 1965);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 0U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/brad/bit-matlab-sim/init_func.m", 1U, 1U, 3U, 0U, 0U,
                  0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 1U, 0U, "init_func", 0, -1, 3126);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 2U, 1864, -1, 3116);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 1U, 1684, -1, 1858);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 0U, 313, -1, 1667);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 1U, 0U, 1668, 1679, 1862);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 1U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/brad/bit-matlab-sim/Miscellaneous/xmat.m", 2U, 1U, 1U,
                  0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
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
                  "/home/brad/bit-matlab-sim/Plant_functions/bit_propagator.m",
                  3U, 1U, 8U, 3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 3U, 0U, "bit_propagator", 0, -1, 1919);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 3U, 7U, 1878, -1, 1910);
  covrtBasicBlockInit(&emlrtCoverageInstance, 3U, 5U, 1504, -1, 1662);
  covrtBasicBlockInit(&emlrtCoverageInstance, 3U, 6U, 1690, -1, 1852);
  covrtBasicBlockInit(&emlrtCoverageInstance, 3U, 4U, 1394, -1, 1445);
  covrtBasicBlockInit(&emlrtCoverageInstance, 3U, 3U, 1155, -1, 1332);
  covrtBasicBlockInit(&emlrtCoverageInstance, 3U, 2U, 462, -1, 1121);
  covrtBasicBlockInit(&emlrtCoverageInstance, 3U, 1U, 419, -1, 437);
  covrtBasicBlockInit(&emlrtCoverageInstance, 3U, 0U, 209, -1, 348);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 3U, 0U, 391, 410, -1, 445);
  covrtIfInit(&emlrtCoverageInstance, 3U, 1U, 1127, 1146, -1, 1872);
  covrtIfInit(&emlrtCoverageInstance, 3U, 2U, 1458, 1487, 1863, 1864);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  covrtWhileInit(&emlrtCoverageInstance, 3U, 0U, 1342, 1368, 1864);
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 3U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/brad/bit-matlab-sim/Plant_functions/"
                  "compute_potential_energy_term.m",
                  4U, 1U, 14U, 3U, 0U, 0U, 0U, 7U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 4U, 0U, "compute_potential_energy_term",
               0, -1, 2671);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 13U, 2644, -1, 2667);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 12U, 2515, -1, 2607);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 11U, 2341, -1, 2368);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 10U, 2259, -1, 2291);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 9U, 1936, -1, 1951);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 8U, 1803, -1, 1837);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 7U, 1715, -1, 1742);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 6U, 1633, -1, 1665);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 5U, 1367, -1, 1382);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 4U, 1092, -1, 1132);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 3U, 1017, -1, 1036);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 2U, 853, -1, 897);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 1U, 560, -1, 712);
  covrtBasicBlockInit(&emlrtCoverageInstance, 4U, 0U, 382, -1, 527);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 4U, 0U, 1066, 1079, -1, 2635);
  covrtIfInit(&emlrtCoverageInstance, 4U, 1U, 1597, 1608, 1686, 1766);
  covrtIfInit(&emlrtCoverageInstance, 4U, 2U, 2223, 2234, 2312, 2392);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 4U, 0U, 537, 551, 720);
  covrtForInit(&emlrtCoverageInstance, 4U, 1U, 829, 843, 2671);
  covrtForInit(&emlrtCoverageInstance, 4U, 2U, 990, 1004, 1048);
  covrtForInit(&emlrtCoverageInstance, 4U, 3U, 1337, 1350, 1853);
  covrtForInit(&emlrtCoverageInstance, 4U, 4U, 1416, 1429, 1786);
  covrtForInit(&emlrtCoverageInstance, 4U, 5U, 1908, 1919, 2623);
  covrtForInit(&emlrtCoverageInstance, 4U, 6U, 1985, 1996, 2412);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 4U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/brad/bit-matlab-sim/Miscellaneous/axis2rot.m", 5U, 1U,
                  4U, 1U, 0U, 0U, 0U, 2U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 5U, 0U, "axis2rot", 0, -1, 718);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 5U, 3U, 545, -1, 678);
  covrtBasicBlockInit(&emlrtCoverageInstance, 5U, 2U, 390, -1, 439);
  covrtBasicBlockInit(&emlrtCoverageInstance, 5U, 1U, 324, -1, 349);
  covrtBasicBlockInit(&emlrtCoverageInstance, 5U, 0U, 190, -1, 270);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 5U, 0U, 362, 373, 452, 694);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 5U, 0U, 280, 291, 714);
  covrtForInit(&emlrtCoverageInstance, 5U, 1U, 300, 311, 706);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 5U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/brad/bit-matlab-sim/Plant_functions/RW_terms.m", 6U,
                  1U, 6U, 4U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 6U, 0U, "RW_terms", 0, -1, 677);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 5U, 509, -1, 562);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 4U, 483, -1, 495);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 3U, 408, -1, 420);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 2U, 334, -1, 357);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 1U, 249, -1, 323);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 0U, 216, -1, 232);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 6U, 0U, 359, 380, 429, 504);
  covrtIfInit(&emlrtCoverageInstance, 6U, 1U, 385, 399, -1, 428);
  covrtIfInit(&emlrtCoverageInstance, 6U, 2U, 429, 455, -1, 504);
  covrtIfInit(&emlrtCoverageInstance, 6U, 3U, 460, 474, -1, 503);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 6U, 0U, 233, 244, 331);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 6U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/brad/bit-matlab-sim/mass_mat_func.m", 7U, 5U, 5U, 0U,
                  0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 7U, 0U, "mass_mat_func", 1, -1, 12710);
  covrtFcnInit(&emlrtCoverageInstance, 7U, 1U, "ft_1", 12711, -1, 34767);
  covrtFcnInit(&emlrtCoverageInstance, 7U, 2U, "ft_2", 34768, -1, 63990);
  covrtFcnInit(&emlrtCoverageInstance, 7U, 3U, "ft_3", 63991, -1, 95534);
  covrtFcnInit(&emlrtCoverageInstance, 7U, 4U, "ft_4", 95535, -1, 121692);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 0U, 193, -1, 12706);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 1U, 12740, -1, 34763);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 2U, 34797, -1, 63986);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 3U, 64020, -1, 95530);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 4U, 95564, -1, 121688);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 7U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/brad/bit-matlab-sim/compute_angular_velocity_C.m", 8U,
                  1U, 3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 8U, 0U, "compute_angular_velocity_C", 0,
               -1, 313);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 2U, 288, -1, 308);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 1U, 204, -1, 278);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 0U, 135, -1, 187);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 8U, 0U, 188, 199, 286);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 8U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/brad/bit-matlab-sim/compute_rotation_mat_C.m", 9U, 1U,
                  3U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 9U, 0U, "compute_rotation_mat_C", 0, -1,
               214);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 9U, 2U, 203, -1, 210);
  covrtBasicBlockInit(&emlrtCoverageInstance, 9U, 1U, 161, -1, 198);
  covrtBasicBlockInit(&emlrtCoverageInstance, 9U, 0U, 131, -1, 144);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 9U, 0U, 145, 156, 202);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 9U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/brad/bit-matlab-sim/rot2axis_C.m", 10U, 1U, 1U, 0U, 0U,
                  0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 10U, 0U, "rot2axis_C", 0, -1, 204);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 10U, 0U, 37, -1, 200);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 10U);
}

void bit_one_step_mex_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    bit_one_step_mex_once();
  }
}

/* End of code generation (bit_one_step_mex_initialize.c) */
