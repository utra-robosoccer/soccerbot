/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 *
 * File: ert_main.c
 *
 * Code generated for Simulink model 'simplepid'.
 *
 * Model version                  : 1.6
 * Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
 * C/C++ source code generated on : Sat Aug 26 16:19:15 2017
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. Traceability
 *    3. Safety precaution
 *    4. RAM efficiency
 * Validation result: Not run
 */

#include <stddef.h>
#include <stdio.h>                     /* This ert_main.c example uses printf/fflush */
#include "simplepid.h"                 /* Model's header file */
#include "rtwtypes.h"

/*
 * Associating rt_OneStep with a real-time clock or interrupt service routine
 * is what makes the generated code "real-time".  The function rt_OneStep is
 * always associated with the base rate of the model.  Subrates are managed
 * by the base rate from inside the generated code.  Enabling/disabling
 * interrupts and floating point context switches are target specific.  This
 * example code indicates where these should take place relative to executing
 * the generated code step function.  Overrun behavior should be tailored to
 * your application needs.  This example simply sets an error status in the
 * real-time model and returns from rt_OneStep.
 */
//void rt_OneStep(void);
//void rt_OneStep(void)
//{
//  static boolean_T OverrunFlag = false;
//
//  /* '<Root>/Position' */
//  static real_T arg_Position = 0.0;
//
//  /* '<Root>/Torque' */
//  static real_T arg_Torque;
//
//  /* Disable interrupts here */
//
//  /* Check for overrun */
//  if (OverrunFlag) {
//    return;
//  }
//
//  OverrunFlag = true;
//
//  /* Save FPU context here (if necessary) */
//  /* Re-enable timer or interrupt here */
//  /* Set model inputs here */
//  arg_Position =
//
//  /* Step the model */
//  simplepid_step(arg_Position, &arg_Torque);
//
//  /* Get model outputs here */
//
//  /* Indicate task complete */
//  OverrunFlag = false;
//
//  /* Disable interrupts here */
//  /* Restore FPU context here (if necessary) */
//  /* Enable interrupts here */
//}

/*
 * The example "main" function illustrates what is required by your
 * application code to initialize, execute, and terminate the generated code.
 * Attaching rt_OneStep to a real-time clock is target specific.  This example
 * illustrates how you do this relative to initializing the model.
 */
//int_T main(int_T argc, const char *argv[])
//{
//  /* Unused arguments */
//  (void)(argc);
//  (void)(argv);
//
//  /* Initialize model */
//  simplepid_initialize();
//
//  /* Attach rt_OneStep to a timer or interrupt service routine with
//   * period 0.01 seconds (the model's base sample time) here.  The
//   * call syntax for rt_OneStep is
//   *
//   *  rt_OneStep();
//   */
//  printf("Warning: The simulation will run forever. "
//         "Generated ERT main won't simulate model step behavior. "
//         "To change this behavior select the 'MAT-file logging' option.\n");
//  fflush((NULL));
//  while (1) {
//    /*  Perform other application tasks here */
//  }
//
//  /* The option 'Remove error status field in real-time model data structure'
//   * is selected, therefore the following code does not need to execute.
//   */
//#if 0
//
//  /* Disable rt_OneStep() here */
//
//  /* Terminate model */
//  simplepid_terminate();
//
//#endif
//
//  return 0;
//}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
