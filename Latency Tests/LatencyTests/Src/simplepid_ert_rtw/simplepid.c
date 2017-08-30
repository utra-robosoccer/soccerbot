/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 *
 * File: simplepid.c
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

#include "simplepid.h"
#include "simplepid_private.h"

/* Block signals and states (auto storage) */
DW_simplepid_T simplepid_DW;

/* Model step function */
void simplepid_step(real_T arg_Position, real_T *arg_Torque)
{
  real_T rtb_FilterCoefficient;
  uint16_T rtb_FixPtSum1;
  real_T rtb_Sum2;

  /* Sum: '<S1>/Sum2' incorporates:
   *  Constant: '<S2>/Vector'
   *  Inport: '<Root>/Position'
   *  MultiPortSwitch: '<S2>/Output'
   *  UnitDelay: '<S3>/Output'
   */
  rtb_Sum2 = simplepid_ConstP.Vector_Value[simplepid_DW.Output_DSTATE] -
    arg_Position;

  /* Gain: '<S1>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S1>/Filter'
   *  Gain: '<S1>/Derivative Gain'
   *  Sum: '<S1>/SumD'
   */
  rtb_FilterCoefficient = (0.0 - simplepid_DW.Filter_DSTATE) * 100.0;

  /* Outport: '<Root>/Torque' incorporates:
   *  Constant: '<S2>/Vector'
   *  DiscreteIntegrator: '<S1>/Integrator'
   *  Inport: '<Root>/Position'
   *  MultiPortSwitch: '<S2>/Output'
   *  Sum: '<S1>/Sum'
   *  Sum: '<S1>/Sum1'
   *  UnitDelay: '<S3>/Output'
   */
  *arg_Torque = ((simplepid_ConstP.Vector_Value[simplepid_DW.Output_DSTATE] -
                  arg_Position) + simplepid_DW.Integrator_DSTATE) +
    rtb_FilterCoefficient;

  /* Sum: '<S4>/FixPt Sum1' incorporates:
   *  Constant: '<S4>/FixPt Constant'
   *  UnitDelay: '<S3>/Output'
   */
  rtb_FixPtSum1 = (uint16_T)(simplepid_DW.Output_DSTATE + 1U);

  /* Switch: '<S5>/FixPt Switch' */
  if (rtb_FixPtSum1 > 256) {
    /* Update for UnitDelay: '<S3>/Output' incorporates:
     *  Constant: '<S5>/Constant'
     */
    simplepid_DW.Output_DSTATE = 0U;
  } else {
    /* Update for UnitDelay: '<S3>/Output' */
    simplepid_DW.Output_DSTATE = rtb_FixPtSum1;
  }

  /* End of Switch: '<S5>/FixPt Switch' */

  /* Update for DiscreteIntegrator: '<S1>/Integrator' */
  simplepid_DW.Integrator_DSTATE += 0.01 * rtb_Sum2;

  /* Update for DiscreteIntegrator: '<S1>/Filter' */
  simplepid_DW.Filter_DSTATE += 0.01 * rtb_FilterCoefficient;
}

/* Model initialize function */
void simplepid_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void simplepid_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
