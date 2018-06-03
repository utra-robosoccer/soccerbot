//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: soccerbot.h
//
// Code generated for Simulink model 'soccerbot'.
//
// Model version                  : 1.19
// Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
// C/C++ source code generated on : Sun Jun  3 00:59:48 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_soccerbot_h_
#define RTW_HEADER_soccerbot_h_
#include <string.h>
#include <float.h>
#include <stddef.h>
#ifndef soccerbot_COMMON_INCLUDES_
# define soccerbot_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "dt_info.h"
#include "ext_work.h"
#include "slros_initialize.h"
#endif                                 // soccerbot_COMMON_INCLUDES_

#include "soccerbot_types.h"

// Shared type includes
#include "multiword_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
# define rtmGetRTWExtModeInfo(rtm)     ((rtm)->extModeInfo)
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

// Block signals (auto storage)
typedef struct {
  uint8_T rawImage_data[230400];
  uint8_T fullImage[230400];
  SL_Bus_soccerbot_sensor_msgs_CompressedImage b_varargout_2;
  char_T tmp_data[128];
  uint8_T busstruct_Format_data[128];
  int32_T rawImage_size[3];
  real_T Gain;                         // '<S2>/Gain'
  real_T Gain1;                        // '<S2>/Gain1'
  int32_T busstruct_Format_size[2];
} B_soccerbot_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  robotics_slros_internal_blo_b_T obj; // '<S5>/Read Image'
  struct {
    void *LoggedData[2];
  } Scope_PWORK;                       // '<Root>/Scope'

  ExampleHelperSimulationRateCo_T obj_d;// '<Root>/MATLAB System'
  robotics_slros_internal_block_T obj_e;// '<S6>/SourceBlock'
  int8_T EnabledSubsystem_SubsysRanBC; // '<S6>/Enabled Subsystem'
  int8_T SelectImages_SubsysRanBC;     // '<Root>/Select Images'
} DW_soccerbot_T;

// Parameters (auto storage)
struct P_soccerbot_T_ {
  real_T ctrlParam1;                   // Variable: ctrlParam1
                                       //  Referenced by: '<S2>/Gain'

  real_T ctrlParam2;                   // Variable: ctrlParam2
                                       //  Referenced by: '<S2>/Gain1'

  SL_Bus_soccerbot_sensor_msgs_CompressedImage Out1_Y0;// Computed Parameter: Out1_Y0
                                                       //  Referenced by: '<S8>/Out1'

  SL_Bus_soccerbot_sensor_msgs_CompressedImage Constant_Value;// Computed Parameter: Constant_Value
                                                              //  Referenced by: '<S6>/Constant'

  uint8_T dataOut_Y0;                  // Computed Parameter: dataOut_Y0
                                       //  Referenced by: '<S5>/dataOut'

};

// Real-time Model Data Structure
struct tag_RTM_soccerbot_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    uint32_T checksums[4];
  } Sizes;

  //
  //  SpecialInfo:
  //  The following substructure contains special information
  //  related to other components that are dependent on RTW.

  struct {
    const void *mappingInfo;
  } SpecialInfo;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

// Block parameters (auto storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_soccerbot_T soccerbot_P;

#ifdef __cplusplus

}
#endif

// Block signals (auto storage)
extern B_soccerbot_T soccerbot_B;

// Block states (auto storage)
extern DW_soccerbot_T soccerbot_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void soccerbot_initialize(void);
  extern void soccerbot_step(void);
  extern void soccerbot_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_soccerbot_T *const soccerbot_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S1>/Constant' : Unused code path elimination
//  Block '<Root>/Bus Assignment' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'soccerbot'
//  '<S1>'   : 'soccerbot/Blank Message'
//  '<S2>'   : 'soccerbot/Control Algorithm'
//  '<S3>'   : 'soccerbot/Perception Algorithm'
//  '<S4>'   : 'soccerbot/Publish'
//  '<S5>'   : 'soccerbot/Select Images'
//  '<S6>'   : 'soccerbot/Subscribe'
//  '<S7>'   : 'soccerbot/Perception Algorithm/MATLAB Function'
//  '<S8>'   : 'soccerbot/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_soccerbot_h_

//
// File trailer for generated code.
//
// [EOF]
//
