//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: soccer_strategy.h
//
// Code generated for Simulink model 'soccer_strategy'.
//
// Model version                  : 1.641
// Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
// C/C++ source code generated on : Sat Jul 14 19:27:01 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Emulation hardware selection:
//    Differs from embedded hardware (MATLAB Host)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_soccer_strategy_h_
#define RTW_HEADER_soccer_strategy_h_
#include <math.h>
#include <string.h>
#include <float.h>
#include <stddef.h>
#ifndef soccer_strategy_COMMON_INCLUDES_
# define soccer_strategy_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "dt_info.h"
#include "ext_work.h"
#include "slros_initialize.h"
#endif                                 // soccer_strategy_COMMON_INCLUDES_

#include "soccer_strategy_types.h"

// Shared type includes
#include "multiword_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

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

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               (&(rtm)->Timing.taskTime0)
#endif

// Block signals for system '<S14>/Moving Average'
typedef struct {
  real_T csumrev[59];
  real_T MovingAverage;                // '<S14>/Moving Average'
} B_MovingAverage_soccer_strategy_T;

// Block states (default storage) for system '<S14>/Moving Average'
typedef struct {
  dsp_private_SlidingWindowAverageCG_soccer_strategy_T gobj_0;// '<S14>/Moving Average' 
  dsp_private_SlidingWindowAverageCG_soccer_strategy_T gobj_1;// '<S14>/Moving Average' 
  dsp_simulink_MovingAverage_soccer_strategy_T obj;// '<S14>/Moving Average'
  boolean_T objisempty;                // '<S14>/Moving Average'
} DW_MovingAverage_soccer_strategy_T;

// Block signals (default storage)
typedef struct {
  SL_Bus_soccer_strategy_team_communication_game_state b_varargout_2;
  SL_Bus_soccer_strategy_sensor_msgs_Imu In1;// '<S40>/In1'
  SL_Bus_soccer_strategy_sensor_msgs_Imu b_varargout_2_m;
  real_T dv0[40];
  SL_Bus_soccer_strategy_soccer_msgs_RobotGoal BusAssignment;// '<Root>/Bus Assignment' 
  real_T VectorConcatenate[20];        // '<S25>/Vector Concatenate'
  real_T Merge[20];                    // '<S26>/Merge'
  real32_T Add[20];                    // '<S2>/Add'
  real_T Constant3;                    // '<S3>/Constant3'
  real_T Constant2;                    // '<S3>/Constant2'
  real_T Delay;                        // '<S4>/Delay'
  real_T Constant2_h;                  // '<Root>/Constant2'
  real_T Constant3_j;                  // '<Root>/Constant3'
  real_T Constant4;                    // '<Root>/Constant4'
  real_T Constant5;                    // '<Root>/Constant5'
  real_T Merge1;                       // '<S4>/Merge1'
  real_T trajAction;                   // '<Root>/Robot Decision'
  real_T wayPoints[25];                // '<Root>/Robot Decision'
  real_T trigger;                      // '<Root>/Robot Decision'
  real_T fallenFront;                  // '<S3>/MATLAB Function'
  real_T fallenBack;                   // '<S3>/MATLAB Function'
  real_T angle[3];                     // '<S14>/Complementary Filter'
  real_T Switch;                       // '<Root>/Switch'
  real32_T DiscreteTimeIntegrator;     // '<S10>/Discrete-Time Integrator'
  real32_T Saturation;                 // '<S10>/Saturation'
  real32_T PermuteMatrix[20];          // '<S2>/Permute Matrix'
  real32_T Sum;                        // '<S17>/Sum'
  real32_T Sum_b;                      // '<S22>/Sum'
  real32_T Sum_c;                      // '<S21>/Sum'
  real32_T FixPtSwitch_k;              // '<S23>/FixPt Switch'
  real32_T Sum_bx;                     // '<S10>/Sum'
  real32_T TSamp;                      // '<S20>/TSamp'
  int32_T idx;
  B_MovingAverage_soccer_strategy_T MovingAverage5;// '<S14>/Moving Average'
  B_MovingAverage_soccer_strategy_T MovingAverage4;// '<S14>/Moving Average'
  B_MovingAverage_soccer_strategy_T MovingAverage3;// '<S14>/Moving Average'
  B_MovingAverage_soccer_strategy_T MovingAverage1;// '<S14>/Moving Average'
  B_MovingAverage_soccer_strategy_T MovingAverage2;// '<S14>/Moving Average'
  B_MovingAverage_soccer_strategy_T MovingAverage;// '<S14>/Moving Average'
} B_soccer_strategy_T;

// Block states (default storage) for system '<Root>'
typedef struct {
  robotics_slros_internal_block_Publisher_soccer_strategy_T obj;// '<S5>/SinkBlock' 
  robotics_slros_internal_block_Subscriber_soccer_strategy_T obj_e;// '<S8>/SourceBlock' 
  robotics_slros_internal_block_Subscriber_soccer_strategy_T obj_g;// '<S7>/SourceBlock' 
  real_T Delay_DSTATE;                 // '<Root>/Delay'
  real_T Delay_DSTATE_g;               // '<S4>/Delay'
  real_T Delay_DSTATE_l[3];            // '<S14>/Delay'
  struct {
    void *LoggedData;
  } Trajectories_PWORK;                // '<Root>/Trajectories'

  struct {
    void *LoggedData[2];
  } Scope3_PWORK;                      // '<S3>/Scope3'

  struct {
    void *LoggedData;
  } Scope_PWORK;                       // '<S10>/Scope'

  struct {
    void *LoggedData[2];
  } Scope4_PWORK;                      // '<S3>/Scope4'

  struct {
    void *LoggedData;
  } Scope1_PWORK;                      // '<S14>/Scope1'

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace_PWORK;               // '<S34>/From Workspace'

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace_PWORK_a;             // '<S37>/From Workspace'

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace_PWORK_k;             // '<S38>/From Workspace'

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace_PWORK_aq;            // '<S39>/From Workspace'

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace_PWORK_i;             // '<S35>/From Workspace'

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace_PWORK_kx;            // '<S36>/From Workspace'

  real32_T Integrator_DSTATE;          // '<S18>/Integrator'
  real32_T UD_DSTATE;                  // '<S20>/UD'
  real32_T DiscreteTimeIntegrator_DSTATE;// '<S10>/Discrete-Time Integrator'
  real32_T Integrator_DSTATE_d;        // '<S21>/Integrator'
  real32_T Filter_DSTATE;              // '<S21>/Filter'
  real32_T Integrator_DSTATE_d5;       // '<S22>/Integrator'
  real32_T Filter_DSTATE_g;            // '<S22>/Filter'
  real32_T UD_DSTATE_e;                // '<S19>/UD'
  real32_T Integrator_DSTATE_o;        // '<S17>/Integrator'
  int32_T sfEvent;                     // '<Root>/Robot Decision'
  ExampleHelperSimulationRateControl_soccer_strategy_T obj_d;// '<Root>/MATLAB System' 
  struct {
    int_T PrevIndex;
  } FromWorkspace_IWORK;               // '<S34>/From Workspace'

  struct {
    int_T PrevIndex;
  } FromWorkspace_IWORK_n;             // '<S37>/From Workspace'

  struct {
    int_T PrevIndex;
  } FromWorkspace_IWORK_j;             // '<S38>/From Workspace'

  struct {
    int_T PrevIndex;
  } FromWorkspace_IWORK_k;             // '<S39>/From Workspace'

  struct {
    int_T PrevIndex;
  } FromWorkspace_IWORK_np;            // '<S35>/From Workspace'

  struct {
    int_T PrevIndex;
  } FromWorkspace_IWORK_g;             // '<S36>/From Workspace'

  uint16_T temporalCounter_i1;         // '<Root>/Robot Decision'
  uint16_T temporalCounter_i2;         // '<Root>/Robot Decision'
  int8_T EnabledSubsystem_SubsysRanBC; // '<S8>/Enabled Subsystem'
  int8_T EnabledSubsystem_SubsysRanBC_f;// '<S7>/Enabled Subsystem'
  int8_T Poses_SubsysRanBC;            // '<S4>/Poses'
  int8_T FixedTrajectories_SubsysRanBC;// '<S4>/Fixed Trajectories'
  int8_T CustomTrajectory_SubsysRanBC; // '<S26>/Custom Trajectory'
  int8_T LookForBall_SubsysRanBC;      // '<S26>/Look For Ball'
  int8_T ReadyToStanding_SubsysRanBC;  // '<S26>/Ready To Standing'
  int8_T StandingToReady_SubsysRanBC;  // '<S26>/Standing To Ready'
  int8_T GetUpBackSmooth_SubsysRanBC;  // '<S26>/Get Up Back Smooth'
  int8_T GetUpFrontSmooth_SubsysRanBC; // '<S26>/Get Up Front Smooth'
  int8_T DynamicTrajectories_SubsysRanBC;// '<S4>/Dynamic Trajectories'
  uint8_T is_active_c3_soccer_strategy;// '<Root>/Robot Decision'
  uint8_T is_c3_soccer_strategy;       // '<Root>/Robot Decision'
  uint8_T is_StatusOk;                 // '<Root>/Robot Decision'
  boolean_T objisempty;                // '<S8>/SourceBlock'
  boolean_T objisempty_i;              // '<S7>/SourceBlock'
  boolean_T objisempty_o;              // '<S5>/SinkBlock'
  boolean_T objisempty_c;              // '<Root>/MATLAB System'
  DW_MovingAverage_soccer_strategy_T MovingAverage5;// '<S14>/Moving Average'
  DW_MovingAverage_soccer_strategy_T MovingAverage4;// '<S14>/Moving Average'
  DW_MovingAverage_soccer_strategy_T MovingAverage3;// '<S14>/Moving Average'
  DW_MovingAverage_soccer_strategy_T MovingAverage1;// '<S14>/Moving Average'
  DW_MovingAverage_soccer_strategy_T MovingAverage2;// '<S14>/Moving Average'
  DW_MovingAverage_soccer_strategy_T MovingAverage;// '<S14>/Moving Average'
} DW_soccer_strategy_T;

// Parameters (default storage)
struct P_soccer_strategy_T_ {
  real_T ready[20];                    // Variable: ready
                                       //  Referenced by: '<S27>/Constant1'

  real_T standing[20];                 // Variable: standing
                                       //  Referenced by: '<S27>/Constant'

  real32_T motorCalibration[20];       // Variable: motorCalibration
                                       //  Referenced by: '<S2>/Calibration'

  real32_T PIDController1_D;           // Mask Parameter: PIDController1_D
                                       //  Referenced by: '<S18>/Derivative Gain'

  real32_T PIDController_D;            // Mask Parameter: PIDController_D
                                       //  Referenced by: '<S21>/Derivative Gain'

  real32_T PIDController1_D_b;         // Mask Parameter: PIDController1_D_b
                                       //  Referenced by: '<S22>/Derivative Gain'

  real32_T PIDController_D_e;          // Mask Parameter: PIDController_D_e
                                       //  Referenced by: '<S17>/Derivative Gain'

  real32_T PIDController_I;            // Mask Parameter: PIDController_I
                                       //  Referenced by: '<S17>/Integral Gain'

  real32_T PIDController1_I;           // Mask Parameter: PIDController1_I
                                       //  Referenced by: '<S18>/Integral Gain'

  real32_T PIDController_I_k;          // Mask Parameter: PIDController_I_k
                                       //  Referenced by: '<S21>/Integral Gain'

  real32_T PIDController1_I_c;         // Mask Parameter: PIDController1_I_c
                                       //  Referenced by: '<S22>/Integral Gain'

  real32_T PIDController_N;            // Mask Parameter: PIDController_N
                                       //  Referenced by: '<S21>/Filter Coefficient'

  real32_T PIDController1_N;           // Mask Parameter: PIDController1_N
                                       //  Referenced by: '<S22>/Filter Coefficient'

  real32_T PIDController1_P;           // Mask Parameter: PIDController1_P
                                       //  Referenced by: '<S18>/Proportional Gain'

  real32_T PIDController_P;            // Mask Parameter: PIDController_P
                                       //  Referenced by: '<S21>/Proportional Gain'

  real32_T PIDController1_P_h;         // Mask Parameter: PIDController1_P_h
                                       //  Referenced by: '<S22>/Proportional Gain'

  real32_T PIDController_P_n;          // Mask Parameter: PIDController_P_n
                                       //  Referenced by: '<S17>/Proportional Gain'

  real32_T WrapToZero1_Threshold;      // Mask Parameter: WrapToZero1_Threshold
                                       //  Referenced by: '<S24>/FixPt Switch'

  real32_T WrapToZero_Threshold;       // Mask Parameter: WrapToZero_Threshold
                                       //  Referenced by: '<S23>/FixPt Switch'

  SL_Bus_soccer_strategy_team_communication_game_state Out1_Y0;// Computed Parameter: Out1_Y0
                                                               //  Referenced by: '<S41>/Out1'

  SL_Bus_soccer_strategy_team_communication_game_state Constant_Value;// Computed Parameter: Constant_Value
                                                                      //  Referenced by: '<S8>/Constant'

  SL_Bus_soccer_strategy_sensor_msgs_Imu Out1_Y0_l;// Computed Parameter: Out1_Y0_l
                                                   //  Referenced by: '<S40>/Out1'

  SL_Bus_soccer_strategy_sensor_msgs_Imu Constant_Value_l;// Computed Parameter: Constant_Value_l
                                                          //  Referenced by: '<S7>/Constant'

  SL_Bus_soccer_strategy_soccer_msgs_RobotGoal Constant_Value_c;// Computed Parameter: Constant_Value_c
                                                                //  Referenced by: '<S1>/Constant'

  real_T Gain_Gain;                    // Expression: 1/180
                                       //  Referenced by: '<S3>/Gain'

  real_T Constant6_Value;              // Expression: 0
                                       //  Referenced by: '<S3>/Constant6'

  real_T Gain1_Gain;                   // Expression: 1/180
                                       //  Referenced by: '<S3>/Gain1'

  real_T Constant7_Value;              // Expression: 0.05
                                       //  Referenced by: '<S3>/Constant7'

  real_T Constant_Value_e[8];          // Expression: [0,0,0,0,0,0,0,0]
                                       //  Referenced by: '<S25>/Constant'

  real_T Step_Time;                    // Expression: getupfrontsmooth.time(end)
                                       //  Referenced by: '<S30>/Step'

  real_T Step_Y0;                      // Expression: 0
                                       //  Referenced by: '<S30>/Step'

  real_T Step_YFinal;                  // Expression: 1
                                       //  Referenced by: '<S30>/Step'

  real_T Step1_Time;                   // Expression: getupbacksmooth.time(end)
                                       //  Referenced by: '<S29>/Step1'

  real_T Step1_Y0;                     // Expression: 0
                                       //  Referenced by: '<S29>/Step1'

  real_T Step1_YFinal;                 // Expression: 1
                                       //  Referenced by: '<S29>/Step1'

  real_T Step_Time_k;                  // Expression: standingtoready.time(end)
                                       //  Referenced by: '<S33>/Step'

  real_T Step_Y0_h;                    // Expression: 0
                                       //  Referenced by: '<S33>/Step'

  real_T Step_YFinal_n;                // Expression: 1
                                       //  Referenced by: '<S33>/Step'

  real_T Step_Time_a;                  // Expression: readytostanding.time(end)
                                       //  Referenced by: '<S32>/Step'

  real_T Step_Y0_p;                    // Expression: 0
                                       //  Referenced by: '<S32>/Step'

  real_T Step_YFinal_a;                // Expression: 1
                                       //  Referenced by: '<S32>/Step'

  real_T Step_Time_c;                  // Expression: headTrajectory.time(end)
                                       //  Referenced by: '<S31>/Step'

  real_T Step_Y0_j;                    // Expression: 0
                                       //  Referenced by: '<S31>/Step'

  real_T Step_YFinal_i;                // Expression: 1
                                       //  Referenced by: '<S31>/Step'

  real_T Step_Time_l;                  // Expression: customTrajectory.time(end)
                                       //  Referenced by: '<S28>/Step'

  real_T Step_Y0_m;                    // Expression: 0
                                       //  Referenced by: '<S28>/Step'

  real_T Step_YFinal_k;                // Expression: 1
                                       //  Referenced by: '<S28>/Step'

  real_T Constant2_Value;              // Expression: 10
                                       //  Referenced by: '<S27>/Constant2'

  real_T OverrideCommand_Value;        // Expression: 12
                                       //  Referenced by: '<Root>/Override Command'

  real_T Delay_InitialCondition;       // Expression: 0.0
                                       //  Referenced by: '<Root>/Delay'

  real_T Switch_Threshold;             // Expression: 0
                                       //  Referenced by: '<Root>/Switch'

  real_T Constant3_Value;              // Expression: 0
                                       //  Referenced by: '<S3>/Constant3'

  real_T Switch2_Threshold;            // Expression: 0
                                       //  Referenced by: '<S3>/Switch2'

  real_T Constant2_Value_l;            // Expression: 0
                                       //  Referenced by: '<S3>/Constant2'

  real_T Switch3_Threshold;            // Expression: 0
                                       //  Referenced by: '<S3>/Switch3'

  real_T Constant_Value_k[20];         // Expression: [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
                                       //  Referenced by: '<S2>/Constant'

  real_T Delay_InitialCondition_k;     // Expression: 0.0
                                       //  Referenced by: '<S4>/Delay'

  real_T Constant2_Value_k;            // Expression: 1
                                       //  Referenced by: '<Root>/Constant2'

  real_T Constant3_Value_n;            // Expression: 0
                                       //  Referenced by: '<Root>/Constant3'

  real_T Constant4_Value;              // Expression: 0
                                       //  Referenced by: '<Root>/Constant4'

  real_T Constant5_Value;              // Expression: 0
                                       //  Referenced by: '<Root>/Constant5'

  real_T Delay_InitialCondition_o[3];  // Expression: [0 0 0]
                                       //  Referenced by: '<S14>/Delay'

  real_T Merge1_InitialOutput;         // Computed Parameter: Merge1_InitialOutput
                                       //  Referenced by: '<S4>/Merge1'

  real32_T Constant_Value_o;           // Computed Parameter: Constant_Value_o
                                       //  Referenced by: '<S23>/Constant'

  real32_T Constant_Value_d;           // Computed Parameter: Constant_Value_d
                                       //  Referenced by: '<S24>/Constant'

  real32_T Constant_Value_f;           // Computed Parameter: Constant_Value_f
                                       //  Referenced by: '<S3>/Constant'

  real32_T DeadZone_Start;             // Computed Parameter: DeadZone_Start
                                       //  Referenced by: '<S10>/Dead Zone'

  real32_T DeadZone_End;               // Computed Parameter: DeadZone_End
                                       //  Referenced by: '<S10>/Dead Zone'

  real32_T Integrator_gainval;         // Computed Parameter: Integrator_gainval
                                       //  Referenced by: '<S18>/Integrator'

  real32_T Integrator_IC;              // Computed Parameter: Integrator_IC
                                       //  Referenced by: '<S18>/Integrator'

  real32_T TSamp_WtEt;                 // Computed Parameter: TSamp_WtEt
                                       //  Referenced by: '<S20>/TSamp'

  real32_T UD_InitialCondition;        // Computed Parameter: UD_InitialCondition
                                       //  Referenced by: '<S20>/UD'

  real32_T Saturation1_UpperSat;       // Computed Parameter: Saturation1_UpperSat
                                       //  Referenced by: '<S10>/Saturation1'

  real32_T Saturation1_LowerSat;       // Computed Parameter: Saturation1_LowerSat
                                       //  Referenced by: '<S10>/Saturation1'

  real32_T Gain2_Gain;                 // Computed Parameter: Gain2_Gain
                                       //  Referenced by: '<S10>/Gain2'

  real32_T DiscreteTimeIntegrator_gainval;// Computed Parameter: DiscreteTimeIntegrator_gainval
                                          //  Referenced by: '<S10>/Discrete-Time Integrator'

  real32_T DiscreteTimeIntegrator_IC;  // Computed Parameter: DiscreteTimeIntegrator_IC
                                       //  Referenced by: '<S10>/Discrete-Time Integrator'

  real32_T Saturation_UpperSat;        // Computed Parameter: Saturation_UpperSat
                                       //  Referenced by: '<S10>/Saturation'

  real32_T Saturation_LowerSat;        // Computed Parameter: Saturation_LowerSat
                                       //  Referenced by: '<S10>/Saturation'

  real32_T Constant1_Value;            // Computed Parameter: Constant1_Value
                                       //  Referenced by: '<S3>/Constant1'

  real32_T Gain2_Gain_n;               // Computed Parameter: Gain2_Gain_n
                                       //  Referenced by: '<S11>/Gain2'

  real32_T Integrator_gainval_e;       // Computed Parameter: Integrator_gainval_e
                                       //  Referenced by: '<S21>/Integrator'

  real32_T Integrator_IC_j;            // Computed Parameter: Integrator_IC_j
                                       //  Referenced by: '<S21>/Integrator'

  real32_T Filter_gainval;             // Computed Parameter: Filter_gainval
                                       //  Referenced by: '<S21>/Filter'

  real32_T Filter_IC;                  // Computed Parameter: Filter_IC
                                       //  Referenced by: '<S21>/Filter'

  real32_T Gain4_Gain;                 // Computed Parameter: Gain4_Gain
                                       //  Referenced by: '<S11>/Gain4'

  real32_T Saturation_UpperSat_h;      // Computed Parameter: Saturation_UpperSat_h
                                       //  Referenced by: '<S11>/Saturation'

  real32_T Saturation_LowerSat_e;      // Computed Parameter: Saturation_LowerSat_e
                                       //  Referenced by: '<S11>/Saturation'

  real32_T Gain3_Gain;                 // Computed Parameter: Gain3_Gain
                                       //  Referenced by: '<S11>/Gain3'

  real32_T Integrator_gainval_d;       // Computed Parameter: Integrator_gainval_d
                                       //  Referenced by: '<S22>/Integrator'

  real32_T Integrator_IC_o;            // Computed Parameter: Integrator_IC_o
                                       //  Referenced by: '<S22>/Integrator'

  real32_T Filter_gainval_d;           // Computed Parameter: Filter_gainval_d
                                       //  Referenced by: '<S22>/Filter'

  real32_T Filter_IC_j;                // Computed Parameter: Filter_IC_j
                                       //  Referenced by: '<S22>/Filter'

  real32_T Gain5_Gain;                 // Computed Parameter: Gain5_Gain
                                       //  Referenced by: '<S11>/Gain5'

  real32_T Saturation1_UpperSat_m;     // Computed Parameter: Saturation1_UpperSat_m
                                       //  Referenced by: '<S11>/Saturation1'

  real32_T Saturation1_LowerSat_d;     // Computed Parameter: Saturation1_LowerSat_d
                                       //  Referenced by: '<S11>/Saturation1'

  real32_T Gain_Gain_o;                // Computed Parameter: Gain_Gain_o
                                       //  Referenced by: '<S11>/Gain'

  real32_T Gain6_Gain;                 // Computed Parameter: Gain6_Gain
                                       //  Referenced by: '<S11>/Gain6'

  real32_T Saturation2_UpperSat;       // Computed Parameter: Saturation2_UpperSat
                                       //  Referenced by: '<S11>/Saturation2'

  real32_T Saturation2_LowerSat;       // Computed Parameter: Saturation2_LowerSat
                                       //  Referenced by: '<S11>/Saturation2'

  real32_T Gain1_Gain_n;               // Computed Parameter: Gain1_Gain_n
                                       //  Referenced by: '<S11>/Gain1'

  real32_T Gain7_Gain;                 // Computed Parameter: Gain7_Gain
                                       //  Referenced by: '<S11>/Gain7'

  real32_T Saturation3_UpperSat;       // Computed Parameter: Saturation3_UpperSat
                                       //  Referenced by: '<S11>/Saturation3'

  real32_T Saturation3_LowerSat;       // Computed Parameter: Saturation3_LowerSat
                                       //  Referenced by: '<S11>/Saturation3'

  real32_T TSamp_WtEt_c;               // Computed Parameter: TSamp_WtEt_c
                                       //  Referenced by: '<S19>/TSamp'

  real32_T UD_InitialCondition_n;      // Computed Parameter: UD_InitialCondition_n
                                       //  Referenced by: '<S19>/UD'

  real32_T Integrator_gainval_i;       // Computed Parameter: Integrator_gainval_i
                                       //  Referenced by: '<S17>/Integrator'

  real32_T Integrator_IC_b;            // Computed Parameter: Integrator_IC_b
                                       //  Referenced by: '<S17>/Integrator'

  uint32_T Delay_DelayLength;          // Computed Parameter: Delay_DelayLength
                                       //  Referenced by: '<Root>/Delay'

  uint32_T UD_DelayLength;             // Computed Parameter: UD_DelayLength
                                       //  Referenced by: '<S20>/UD'

  uint32_T Delay_DelayLength_n;        // Computed Parameter: Delay_DelayLength_n
                                       //  Referenced by: '<S4>/Delay'

  uint32_T UD_DelayLength_d;           // Computed Parameter: UD_DelayLength_d
                                       //  Referenced by: '<S19>/UD'

  uint32_T Delay_DelayLength_g;        // Computed Parameter: Delay_DelayLength_g
                                       //  Referenced by: '<S14>/Delay'

};

// Real-time Model Data Structure
struct tag_RTM_soccer_strategy_T {
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
    uint32_T clockTick1;
    struct {
      uint8_T TID[2];
    } TaskCounters;

    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C" {

#endif

  extern P_soccer_strategy_T soccer_strategy_P;

#ifdef __cplusplus

}
#endif

// Block signals (default storage)
extern B_soccer_strategy_T soccer_strategy_B;

// Block states (default storage)
extern DW_soccer_strategy_T soccer_strategy_DW;

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern void soccer_strategy_initialize(void);
  extern void soccer_strategy_step(void);
  extern void soccer_strategy_terminate(void);

#ifdef __cplusplus

}
#endif

// Real-time Model object
#ifdef __cplusplus

extern "C" {

#endif

  extern RT_MODEL_soccer_strategy_T *const soccer_strategy_M;

#ifdef __cplusplus

}
#endif

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Constant1' : Unused code path elimination
//  Block '<S19>/DTDup' : Unused code path elimination
//  Block '<S20>/DTDup' : Unused code path elimination
//  Block '<S23>/FixPt Data Type Duplicate1' : Unused code path elimination
//  Block '<S24>/FixPt Data Type Duplicate1' : Unused code path elimination
//  Block '<Root>/Rate Transition' : Unused code path elimination


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
//  '<Root>' : 'soccer_strategy'
//  '<S1>'   : 'soccer_strategy/Blank Message'
//  '<S2>'   : 'soccer_strategy/Calibrate'
//  '<S3>'   : 'soccer_strategy/IMU Feedback'
//  '<S4>'   : 'soccer_strategy/Motion Planner'
//  '<S5>'   : 'soccer_strategy/Publish'
//  '<S6>'   : 'soccer_strategy/Robot Decision'
//  '<S7>'   : 'soccer_strategy/Subscribe1'
//  '<S8>'   : 'soccer_strategy/Subscribe2'
//  '<S9>'   : 'soccer_strategy/IMU Feedback/Angle Estimation'
//  '<S10>'  : 'soccer_strategy/IMU Feedback/Forward Backward Control'
//  '<S11>'  : 'soccer_strategy/IMU Feedback/Left Right Control'
//  '<S12>'  : 'soccer_strategy/IMU Feedback/MATLAB Function'
//  '<S13>'  : 'soccer_strategy/IMU Feedback/MATLAB Function1'
//  '<S14>'  : 'soccer_strategy/IMU Feedback/Angle Estimation/Complementary FIlter'
//  '<S15>'  : 'soccer_strategy/IMU Feedback/Angle Estimation/Direct Information'
//  '<S16>'  : 'soccer_strategy/IMU Feedback/Angle Estimation/Complementary FIlter/Complementary Filter'
//  '<S17>'  : 'soccer_strategy/IMU Feedback/Forward Backward Control/PID Controller'
//  '<S18>'  : 'soccer_strategy/IMU Feedback/Forward Backward Control/PID Controller1'
//  '<S19>'  : 'soccer_strategy/IMU Feedback/Forward Backward Control/PID Controller/Differentiator'
//  '<S20>'  : 'soccer_strategy/IMU Feedback/Forward Backward Control/PID Controller1/Differentiator'
//  '<S21>'  : 'soccer_strategy/IMU Feedback/Left Right Control/PID Controller'
//  '<S22>'  : 'soccer_strategy/IMU Feedback/Left Right Control/PID Controller1'
//  '<S23>'  : 'soccer_strategy/IMU Feedback/Left Right Control/Wrap To Zero'
//  '<S24>'  : 'soccer_strategy/IMU Feedback/Left Right Control/Wrap To Zero1'
//  '<S25>'  : 'soccer_strategy/Motion Planner/Dynamic Trajectories'
//  '<S26>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories'
//  '<S27>'  : 'soccer_strategy/Motion Planner/Poses'
//  '<S28>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories/Custom Trajectory'
//  '<S29>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories/Get Up Back Smooth'
//  '<S30>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories/Get Up Front Smooth'
//  '<S31>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories/Look For Ball'
//  '<S32>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories/Ready To Standing'
//  '<S33>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories/Standing To Ready'
//  '<S34>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories/Custom Trajectory/Signal'
//  '<S35>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories/Get Up Back Smooth/Signal1'
//  '<S36>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories/Get Up Front Smooth/Signal'
//  '<S37>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories/Look For Ball/Signal'
//  '<S38>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories/Ready To Standing/Signal'
//  '<S39>'  : 'soccer_strategy/Motion Planner/Fixed Trajectories/Standing To Ready/Signal'
//  '<S40>'  : 'soccer_strategy/Subscribe1/Enabled Subsystem'
//  '<S41>'  : 'soccer_strategy/Subscribe2/Enabled Subsystem'

#endif                                 // RTW_HEADER_soccer_strategy_h_

//
// File trailer for generated code.
//
// [EOF]
//
