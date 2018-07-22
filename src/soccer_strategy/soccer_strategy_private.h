//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: soccer_strategy_private.h
//
// Code generated for Simulink model 'soccer_strategy'.
//
<<<<<<< HEAD
// Model version                  : 1.641
// Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
// C/C++ source code generated on : Sat Jul 14 19:27:01 2018
=======
// Model version                  : 1.643
// Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
// C/C++ source code generated on : Sun Jul 15 22:34:09 2018
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Emulation hardware selection:
//    Differs from embedded hardware (MATLAB Host)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_soccer_strategy_private_h_
#define RTW_HEADER_soccer_strategy_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "soccer_strategy.h"

// Private macros used by the generated code to access rtModel
#ifndef rtmSetTFinal
# define rtmSetTFinal(rtm, val)        ((rtm)->Timing.tFinal = (val))
#endif

// Used by FromWorkspace Block: '<S36>/From Workspace'
#ifndef rtInterpolate
# define rtInterpolate(v1,v2,f1,f2)    (((v1)==(v2))?((double)(v1)): (((f1)*((double)(v1)))+((f2)*((double)(v2)))))
#endif

#ifndef rtRound
# define rtRound(v)                    ( ((v) >= 0) ? floor((v) + 0.5) : ceil((v) - 0.5) )
#endif

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void soccer_strategy_MovingAverage_Init
  (DW_MovingAverage_soccer_strategy_T *localDW);
extern void soccer_strategy_MovingAverage_Start
  (DW_MovingAverage_soccer_strategy_T *localDW);
extern void soccer_strategy_MovingAverage(real_T rtu_0,
  B_MovingAverage_soccer_strategy_T *localB, DW_MovingAverage_soccer_strategy_T *
  localDW);
extern void soccer_strategy_MovingAverage_Term
  (DW_MovingAverage_soccer_strategy_T *localDW);

#endif                                 // RTW_HEADER_soccer_strategy_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
