/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 *
 * File: simplepid.h
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

#ifndef RTW_HEADER_simplepid_h_
#define RTW_HEADER_simplepid_h_
#ifndef simplepid_COMMON_INCLUDES_
# define simplepid_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* simplepid_COMMON_INCLUDES_ */

#include "simplepid_types.h"

/* Macros for accessing real-time model data structure */

/* Block signals and states (auto storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S1>/Integrator' */
  real_T Filter_DSTATE;                /* '<S1>/Filter' */
  uint16_T Output_DSTATE;              /* '<S3>/Output' */
} DW_simplepid_T;

/* Constant parameters (auto storage) */
typedef struct {
  /* Expression: OutValues
   * Referenced by: '<S2>/Vector'
   */
  real_T Vector_Value[257];
} ConstP_simplepid_T;

/* Block signals and states (auto storage) */
extern DW_simplepid_T simplepid_DW;

/* Constant parameters (auto storage) */
extern const ConstP_simplepid_T simplepid_ConstP;

/* Model entry point functions */
extern void simplepid_initialize(void);
extern void simplepid_terminate(void);

/* Customized model step function */
extern void simplepid_step(real_T arg_Position, real_T *arg_Torque);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S3>/Data Type Propagation' : Unused code path elimination
 * Block '<S4>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S5>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S1>/Integral Gain' : Eliminated nontunable gain of 1
 * Block '<S1>/Proportional Gain' : Eliminated nontunable gain of 1
 * Block '<S1>/Setpoint Weighting (Derivative)' : Eliminated nontunable gain of 1
 * Block '<S1>/Setpoint Weighting (Proportional)' : Eliminated nontunable gain of 1
 * Block '<S2>/Out' : Eliminate redundant signal conversion block
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'simplepid'
 * '<S1>'   : 'simplepid/PID Controller (2DOF)'
 * '<S2>'   : 'simplepid/Repeating Sequence Stair'
 * '<S3>'   : 'simplepid/Repeating Sequence Stair/LimitedCounter'
 * '<S4>'   : 'simplepid/Repeating Sequence Stair/LimitedCounter/Increment Real World'
 * '<S5>'   : 'simplepid/Repeating Sequence Stair/LimitedCounter/Wrap To Zero'
 */

/*-
 * Requirements for '<Root>': simplepid
 */
#endif                                 /* RTW_HEADER_simplepid_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
