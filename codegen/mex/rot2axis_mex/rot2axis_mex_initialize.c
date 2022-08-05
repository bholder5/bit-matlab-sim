/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rot2axis_mex_initialize.c
 *
 * Code generation for function 'rot2axis_mex_initialize'
 *
 */

/* Include files */
#include "rot2axis_mex_initialize.h"
#include "_coder_rot2axis_mex_mex.h"
#include "rot2axis_mex_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void rot2axis_mex_once(void);

/* Function Definitions */
static void rot2axis_mex_once(void)
{
  mex_InitInfAndNan();
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/brad/bit-matlab-sim/bit_one_step.m", 0U, 1U, 5U, 1U,
                  0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 0U, 0U, "bit_one_step", 0, -1, 2807);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 4U, 2677, -1, 2782);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 3U, 2641, -1, 2664);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 2U, 2385, -1, 2597);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 1U, 795, -1, 1790);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 0U, 107, -1, 557);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 0U, 0U, 2602, 2632, -1, 2672);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 0U, 0U, 571, 597, 2314);
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
  covrtFcnInit(&emlrtCoverageInstance, 1U, 0U, "init_func", 0, -1, 2528);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 2U, 1962, -1, 2521);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 1U, 1769, -1, 1948);
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 0U, 296, -1, 1744);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 1U, 0U, 1749, 1760, 1956);
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
                  3U, 1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 3U, 0U, "bit_propagator", 0, -1, 1209);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 3U, 0U, 179, -1, 1195);
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
  covrtFcnInit(&emlrtCoverageInstance, 6U, 0U, "RW_terms", 0, -1, 661);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 5U, 546, -1, 656);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 4U, 520, -1, 532);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 3U, 445, -1, 457);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 2U, 371, -1, 394);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 1U, 286, -1, 360);
  covrtBasicBlockInit(&emlrtCoverageInstance, 6U, 0U, 253, -1, 269);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 6U, 0U, 396, 417, 466, 541);
  covrtIfInit(&emlrtCoverageInstance, 6U, 1U, 422, 436, -1, 465);
  covrtIfInit(&emlrtCoverageInstance, 6U, 2U, 466, 492, -1, 541);
  covrtIfInit(&emlrtCoverageInstance, 6U, 3U, 497, 511, -1, 540);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 6U, 0U, 270, 281, 368);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 6U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/brad/bit-matlab-sim/Plant_functions/compute_mass_matrix.m", 7U, 1U,
      9U, 1U, 0U, 0U, 0U, 6U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 7U, 0U, "compute_mass_matrix", 0, -1,
               1589);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 8U, 1539, -1, 1560);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 7U, 1485, -1, 1506);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 6U, 1225, -1, 1464);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 5U, 1121, -1, 1146);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 4U, 910, -1, 933);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 3U, 791, -1, 832);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 2U, 749, -1, 758);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 1U, 511, -1, 665);
  covrtBasicBlockInit(&emlrtCoverageInstance, 7U, 0U, 271, -1, 436);
  /* Initialize If Information */
  covrtIfInit(&emlrtCoverageInstance, 7U, 0U, 1515, 1526, -1, 1572);
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 7U, 0U, 495, 506, 669);
  covrtForInit(&emlrtCoverageInstance, 7U, 1U, 713, 724, 1584);
  covrtForInit(&emlrtCoverageInstance, 7U, 2U, 729, 740, 1580);
  covrtForInit(&emlrtCoverageInstance, 7U, 3U, 767, 778, 1476);
  covrtForInit(&emlrtCoverageInstance, 7U, 4U, 858, 871, 1065);
  covrtForInit(&emlrtCoverageInstance, 7U, 5U, 1091, 1104, 1212);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 7U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(
      &emlrtCoverageInstance,
      "/home/brad/bit-matlab-sim/Plant_functions/compute_rotation_mat.m", 8U,
      1U, 2U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 8U, 0U, "compute_rotation_mat", 0, -1,
               202);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 1U, 157, -1, 194);
  covrtBasicBlockInit(&emlrtCoverageInstance, 8U, 0U, 129, -1, 140);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  covrtForInit(&emlrtCoverageInstance, 8U, 0U, 141, 152, 198);
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 8U);
  /* Allocate instance data */
  covrtAllocateInstanceData(&emlrtCoverageInstance);
  /* Initialize Coverage Information */
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/brad/bit-matlab-sim/Miscellaneous/rot2axis.m", 9U, 1U,
                  1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);
  /* Initialize Function Information */
  covrtFcnInit(&emlrtCoverageInstance, 9U, 0U, "rot2axis", 0, -1, 200);
  /* Initialize Basic Block Information */
  covrtBasicBlockInit(&emlrtCoverageInstance, 9U, 0U, 36, -1, 196);
  /* Initialize If Information */
  /* Initialize MCDC Information */
  /* Initialize For Information */
  /* Initialize While Information */
  /* Initialize Switch Information */
  /* Start callback for coverage engine */
  covrtScriptStart(&emlrtCoverageInstance, 9U);
}

void rot2axis_mex_initialize(void)
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
    rot2axis_mex_once();
  }
}

/* End of code generation (rot2axis_mex_initialize.c) */
