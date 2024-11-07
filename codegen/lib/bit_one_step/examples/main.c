/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 22-Sep-2024 08:32:20
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
#include "bit_one_step.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void argInit_104x1_real_T(double result[104]);

static void argInit_21x1_real_T(double result[21]);

static void argInit_5x1_real_T(double result[5]);

static void argInit_9x1_real_T(double result[9]);

static double argInit_real_T(void);

static unsigned short argInit_uint16_T(void);

static unsigned char argInit_uint8_T(void);

/* Function Definitions */
/*
 * Arguments    : double result[104]
 * Return Type  : void
 */
static void argInit_104x1_real_T(double result[104])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 104; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[21]
 * Return Type  : void
 */
static void argInit_21x1_real_T(double result[21])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 21; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[5]
 * Return Type  : void
 */
static void argInit_5x1_real_T(double result[5])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 5; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[9]
 * Return Type  : void
 */
static void argInit_9x1_real_T(double result[9])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 9; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : unsigned short
 */
static unsigned short argInit_uint16_T(void)
{
  return 0U;
}

/*
 * Arguments    : void
 * Return Type  : unsigned char
 */
static unsigned char argInit_uint8_T(void)
{
  return 0U;
}

/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_bit_one_step();
  /* Terminate the application.
You do not need to do this more than one time. */
  bit_one_step_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_bit_one_step(void)
{
  double dv1[104];
  double y_flex[104];
  double dv[21];
  double y_true[21];
  double b_tau_applied_tmp[9];
  double tau_applied_tmp[9];
  double dv2[5];
  double w_piv_tmp;
  unsigned char piv_flag_tmp;
  /* Initialize function 'bit_one_step' input arguments. */
  /* Initialize function input argument 'x0'. */
  /* Initialize function input argument 'tau_applied'. */
  argInit_9x1_real_T(tau_applied_tmp);
  /* Initialize function input argument 'unlock'. */
  w_piv_tmp = argInit_real_T();
  piv_flag_tmp = argInit_uint8_T();
  /* Initialize function input argument 'x_flex0'. */
  /* Initialize function input argument 'tau_flex'. */
  /* Call the entry-point 'bit_one_step'. */
  argInit_21x1_real_T(dv);
  argInit_104x1_real_T(dv1);
  argInit_5x1_real_T(dv2);
  memcpy(&b_tau_applied_tmp[0], &tau_applied_tmp[0], 9U * sizeof(double));
  bit_one_step(dv, b_tau_applied_tmp, tau_applied_tmp, w_piv_tmp, piv_flag_tmp,
               w_piv_tmp, argInit_uint16_T(), w_piv_tmp, w_piv_tmp, dv1, dv2,
               piv_flag_tmp, piv_flag_tmp, y_true, y_flex);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
