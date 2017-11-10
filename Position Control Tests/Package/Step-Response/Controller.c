/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 *
 * File: Controller.c
 *
 * Code generated for Simulink model 'Controller'.
 *
 * Model version                  : 1.16
 * Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
 * C/C++ source code generated on : Sat Nov 04 04:32:00 2017
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: STMicroelectronics->ST10/Super10
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Controller.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

/* Used by FromWorkspace Block: '<S2>/FromWs' */
#ifndef rtInterpolate
# define rtInterpolate(v1,v2,f1,f2)    (((v1)==(v2))?((double)(v1)): (((f1)*((double)(v1)))+((f2)*((double)(v2)))))
#endif

#ifndef rtRound
# define rtRound(v)                    ( ((v) >= 0) ? floor((v) + 0.5) : ceil((v) - 0.5) )
#endif

/* Block signals and states (auto storage) */
DW rtDW;

/* External outputs (root outports fed by signals with auto storage) */
ExtY rtY;

/* Real-time model */
RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void Controller_step(real_T arg_Position, real_T *arg_Torque)
{
  /* local block i/o variables */
  real_T rtb_FilterCoefficient;
  real_T rtb_IntegralGain;
  real_T Sum1;

  /* FromWorkspace: '<S2>/FromWs' */
  {
    real_T *pDataValues = (real_T *) rtDW.FromWs_PWORK.DataPtr;
    real_T *pTimeValues = (real_T *) rtDW.FromWs_PWORK.TimePtr;
    int_T currTimeIndex = rtDW.FromWs_IWORK.PrevIndex;
    real_T t = ((rtM->Timing.clockTick1) * 0.01);
    if (t >= pTimeValues[3]) {
      rtDW.FromWs = pDataValues[3];
    } else {
      /* Get index */
      if (t <= pTimeValues[0]) {
        currTimeIndex = 0;
      } else if (t >= pTimeValues[3]) {
        currTimeIndex = 2;
      } else {
        if (t < pTimeValues[currTimeIndex]) {
          while (t < pTimeValues[currTimeIndex]) {
            currTimeIndex--;
          }
        } else {
          while (t >= pTimeValues[currTimeIndex + 1]) {
            currTimeIndex++;
          }
        }
      }

      rtDW.FromWs_IWORK.PrevIndex = currTimeIndex;

      /* Post output */
      {
        real_T t1 = pTimeValues[currTimeIndex];
        real_T t2 = pTimeValues[currTimeIndex + 1];
        if (t1 == t2) {
          if (t < t1) {
            rtDW.FromWs = pDataValues[currTimeIndex];
          } else {
            rtDW.FromWs = pDataValues[currTimeIndex + 1];
          }
        } else {
          real_T f1 = (t2 - t) / (t2 - t1);
          real_T f2 = 1.0 - f1;
          real_T d1;
          real_T d2;
          int_T TimeIndex= currTimeIndex;
          d1 = pDataValues[TimeIndex];
          d2 = pDataValues[TimeIndex + 1];
          rtDW.FromWs = (real_T) rtInterpolate(d1, d2, f1, f2);
          pDataValues += 4;
        }
      }
    }
  }

  /* Sum: '<Root>/Sum1' incorporates:
   *  Inport: '<Root>/Position'
   */
  Sum1 = rtDW.FromWs - arg_Position;

  /* Gain: '<S1>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S1>/Filter'
   *  Gain: '<S1>/Derivative Gain'
   *  Sum: '<S1>/SumD'
   */
	// 0.2 here is the derivative gain
   rtb_FilterCoefficient = (0.2 * Sum1 - rtDW.Filter_DSTATE) * 100.0;

  /* Outport: '<Root>/Torque' incorporates:
   *  Constant: '<Root>/Constant'
   *  DiscreteIntegrator: '<S1>/Integrator'
   *  Gain: '<S1>/Proportional Gain'
   *  Rounding: '<Root>/Floor'
   *  Sum: '<Root>/Sum'
   *  Sum: '<S1>/Sum'
   */
	// 5.0 here is the proportional gain
  rtY.Torque = floor(((5.0 * Sum1 + rtDW.Integrator_DSTATE) +
                      rtb_FilterCoefficient) + 512.0);

  /* Gain: '<S1>/Integral Gain' */
	// 10.0 here is the integral gain
  rtb_IntegralGain = 10.0 * Sum1;

  /* Update for DiscreteIntegrator: '<S1>/Integrator' */
  rtDW.Integrator_DSTATE += 0.01 * rtb_IntegralGain;

  /* Update for DiscreteIntegrator: '<S1>/Filter' */
  rtDW.Filter_DSTATE += 0.01 * rtb_FilterCoefficient;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  rtM->Timing.t[0] =
    (++rtM->Timing.clockTick0) * rtM->Timing.stepSize0;

  {
    /* Update absolute timer for sample time: [0.01s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.01, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     */
    rtM->Timing.clockTick1++;
  }

  /* Copy value for root outport '<Root>/Torque' since it is accessed globally */
  *arg_Torque = rtY.Torque;
}

/* Model initialize function */
void Controller_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, ((const char_T **)
      (&rtmGetErrorStatus(rtM))));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&rtM->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.01;

  /* Start for FromWorkspace: '<S2>/FromWs' */
  {
    static real_T pTimeValues0[] = { 0.0, 4.0, 4.0, 10.0 } ;

    static real_T pDataValues0[] = { 0.0, 0.0, 1.5707963267948966,
      1.5707963267948966 } ;

    rtDW.FromWs_PWORK.TimePtr = (void *) pTimeValues0;
    rtDW.FromWs_PWORK.DataPtr = (void *) pDataValues0;
    rtDW.FromWs_IWORK.PrevIndex = 0;
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
