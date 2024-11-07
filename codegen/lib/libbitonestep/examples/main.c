/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 23-Sep-2024 11:45:57
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "libbitonestep.h"
#include "libbitonestep_internal_types.h"
#include <stdio.h>
#include <string.h>

/* Function Declarations */
static void argInit_104x1_real_T(real_T result[104]);

static void argInit_18x1_real_T(real_T result[18]);

static void argInit_21x1_real_T(real_T result[21]);

static void argInit_3x3_real_T(real_T result[3][3]);

static void argInit_3x9_real_T(real_T result[9][3]);

static void argInit_5x1_real_T(real_T result[5]);

static void argInit_9x1_real_T(real_T result[9]);

static boolean_T argInit_boolean_T(void);

static real_T argInit_real_T(void);

static uint16_T argInit_uint16_T(void);

static void b_rtErrorWithMessageID(const char_T *aFcnName, int32_T aLineNum);

/* Function Definitions */
/*
 * Arguments    : real_T result[104]
 * Return Type  : void
 */
static void argInit_104x1_real_T(real_T result[104])
{
  int32_T idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 104; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : real_T result[18]
 * Return Type  : void
 */
static void argInit_18x1_real_T(real_T result[18])
{
  int32_T idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 18; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : real_T result[21]
 * Return Type  : void
 */
static void argInit_21x1_real_T(real_T result[21])
{
  int32_T idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 21; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : real_T result[3][3]
 * Return Type  : void
 */
static void argInit_3x3_real_T(real_T result[3][3])
{
  int32_T idx0;
  int32_T idx1;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    for (idx1 = 0; idx1 < 3; idx1++) {
      /* Set the value of the array element.
Change this value to the value that the application requires. */
      result[idx1][idx0] = argInit_real_T();
    }
  }
}

/*
 * Arguments    : real_T result[9][3]
 * Return Type  : void
 */
static void argInit_3x9_real_T(real_T result[9][3])
{
  int32_T idx0;
  int32_T idx1;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    for (idx1 = 0; idx1 < 9; idx1++) {
      /* Set the value of the array element.
Change this value to the value that the application requires. */
      result[idx1][idx0] = argInit_real_T();
    }
  }
}

/*
 * Arguments    : real_T result[5]
 * Return Type  : void
 */
static void argInit_5x1_real_T(real_T result[5])
{
  int32_T idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 5; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : real_T result[9]
 * Return Type  : void
 */
static void argInit_9x1_real_T(real_T result[9])
{
  int32_T idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 9; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : boolean_T
 */
static boolean_T argInit_boolean_T(void)
{
  return false;
}

/*
 * Arguments    : void
 * Return Type  : real_T
 */
static real_T argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : uint16_T
 */
static uint16_T argInit_uint16_T(void)
{
  return 0U;
}

/*
 * Arguments    : const char_T *aFcnName
 *                int32_T aLineNum
 * Return Type  : void
 */
static void b_rtErrorWithMessageID(const char_T *aFcnName, int32_T aLineNum)
{
  (void)fprintf(stderr,
                "Example main does not support command line arguments.");
  (void)fprintf(stderr, "\n");
  (void)fprintf(stderr, "Error in %s (line %d)", aFcnName, aLineNum);
  (void)fprintf(stderr, "\n");
  (void)fflush(stderr);
  abort();
}

/*
 * Arguments    : int32_T argc
 *                char **argv
 * Return Type  : int32_T
 */
int32_T main(int32_T argc, char **argv)
{
  static rtRunTimeErrorInfo emlrtRTEI = {
      1,             /* lineNo */
      "bit_one_step" /* fName */
  };
  (void)argv;
  if (argc > 1) {
    b_rtErrorWithMessageID(emlrtRTEI.fName, emlrtRTEI.lineNo);
  }
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_bit_one_step();
  main_compute_angular_velocity_C();
  main_compute_angular_velocity_roll_C();
  main_compute_angular_velocity_yaw_C();
  main_compute_rotation_mat_C();
  main_compute_rotation_mat_roll_C();
  main_compute_rotation_mat_yaw_C();
  main_rot2axis_C();
  /* Terminate the application.
You do not need to do this more than one time. */
  libbitonestep_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_bit_one_step(void)
{
  real_T dv1[104];
  real_T y_flex[104];
  real_T dv[21];
  real_T y_true[21];
  real_T b_tau_applied_tmp[9];
  real_T tau_applied_tmp[9];
  real_T dv2[5];
  real_T w_piv_tmp;
  boolean_T piv_flag_tmp;
  /* Initialize function 'bit_one_step' input arguments. */
  /* Initialize function input argument 'x0'. */
  /* Initialize function input argument 'tau_applied'. */
  argInit_9x1_real_T(tau_applied_tmp);
  /* Initialize function input argument 'unlock'. */
  w_piv_tmp = argInit_real_T();
  piv_flag_tmp = argInit_boolean_T();
  /* Initialize function input argument 'x_flex0'. */
  /* Initialize function input argument 'tau_flex'. */
  /* Call the entry-point 'bit_one_step'. */
  argInit_21x1_real_T(dv);
  argInit_104x1_real_T(dv1);
  argInit_5x1_real_T(dv2);
  (void)memcpy(&b_tau_applied_tmp[0], &tau_applied_tmp[0],
               9U * (sizeof(real_T)));
  bit_one_step(dv, b_tau_applied_tmp, tau_applied_tmp, w_piv_tmp, piv_flag_tmp,
               w_piv_tmp, argInit_uint16_T(), w_piv_tmp, w_piv_tmp, dv1, dv2,
               piv_flag_tmp, piv_flag_tmp, y_true, y_flex);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_compute_angular_velocity_C(void)
{
  real_T dv1[9][3];
  real_T dv[18];
  real_T omega[3];
  /* Initialize function 'compute_angular_velocity_C' input arguments. */
  /* Initialize function input argument 'x'. */
  /* Initialize function input argument 'z_n'. */
  /* Call the entry-point 'compute_angular_velocity_C'. */
  argInit_18x1_real_T(dv);
  argInit_3x9_real_T(dv1);
  compute_angular_velocity_C(dv, dv1, omega);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_compute_angular_velocity_roll_C(void)
{
  real_T dv1[9][3];
  real_T dv[18];
  real_T omega[3];
  /* Initialize function 'compute_angular_velocity_roll_C' input arguments. */
  /* Initialize function input argument 'x'. */
  /* Initialize function input argument 'z_n'. */
  /* Call the entry-point 'compute_angular_velocity_roll_C'. */
  argInit_18x1_real_T(dv);
  argInit_3x9_real_T(dv1);
  compute_angular_velocity_roll_C(dv, dv1, omega);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_compute_angular_velocity_yaw_C(void)
{
  real_T dv1[9][3];
  real_T dv[18];
  real_T omega[3];
  /* Initialize function 'compute_angular_velocity_yaw_C' input arguments. */
  /* Initialize function input argument 'x'. */
  /* Initialize function input argument 'z_n'. */
  /* Call the entry-point 'compute_angular_velocity_yaw_C'. */
  argInit_18x1_real_T(dv);
  argInit_3x9_real_T(dv1);
  compute_angular_velocity_yaw_C(dv, dv1, omega);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_compute_rotation_mat_C(void)
{
  real_T dv[9][3];
  real_T C[3][3];
  real_T dv1[9];
  /* Initialize function 'compute_rotation_mat_C' input arguments. */
  /* Initialize function input argument 'z_n'. */
  /* Initialize function input argument 'theta'. */
  /* Call the entry-point 'compute_rotation_mat_C'. */
  argInit_3x9_real_T(dv);
  argInit_9x1_real_T(dv1);
  compute_rotation_mat_C(dv, dv1, C);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_compute_rotation_mat_roll_C(void)
{
  real_T dv[9][3];
  real_T C[3][3];
  real_T dv1[9];
  /* Initialize function 'compute_rotation_mat_roll_C' input arguments. */
  /* Initialize function input argument 'z_n'. */
  /* Initialize function input argument 'theta'. */
  /* Call the entry-point 'compute_rotation_mat_roll_C'. */
  argInit_3x9_real_T(dv);
  argInit_9x1_real_T(dv1);
  compute_rotation_mat_roll_C(dv, dv1, C);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_compute_rotation_mat_yaw_C(void)
{
  real_T dv[9][3];
  real_T C[3][3];
  real_T dv1[9];
  /* Initialize function 'compute_rotation_mat_yaw_C' input arguments. */
  /* Initialize function input argument 'z_n'. */
  /* Initialize function input argument 'theta'. */
  /* Call the entry-point 'compute_rotation_mat_yaw_C'. */
  argInit_3x9_real_T(dv);
  argInit_9x1_real_T(dv1);
  compute_rotation_mat_yaw_C(dv, dv1, C);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_rot2axis_C(void)
{
  real_T dv[3][3];
  real_T v[3];
  real_T phi;
  /* Initialize function 'rot2axis_C' input arguments. */
  /* Initialize function input argument 'C'. */
  /* Call the entry-point 'rot2axis_C'. */
  argInit_3x3_real_T(dv);
  rot2axis_C(dv, v, &phi);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
