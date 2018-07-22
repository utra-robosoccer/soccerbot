//
//  soccer_strategy_dt.h
//
//  Sponsored License - for use in support of a program or activity
//  sponsored by MathWorks.  Not for government, commercial or other
//  non-sponsored organizational use.
//
//  Code generation for model "soccer_strategy".
//
<<<<<<< HEAD
//  Model version              : 1.641
//  Simulink Coder version : 8.14 (R2018a) 06-Feb-2018
//  C++ source code generated on : Sat Jul 14 19:27:01 2018
=======
//  Model version              : 1.643
//  Simulink Coder version : 8.14 (R2018a) 06-Feb-2018
//  C++ source code generated on : Sun Jul 15 22:34:09 2018
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
//
//  Target selection: ert.tlc
//  Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
//  Emulation hardware selection:
//     Differs from embedded hardware (MATLAB Host)
//  Code generation objectives: Unspecified
//  Validation result: Not run


#include "ext_types.h"

// data type size table
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T),
  sizeof(SL_Bus_ROSVariableLengthArrayInfo),
  sizeof(SL_Bus_soccer_strategy_ros_time_Time),
  sizeof(SL_Bus_soccer_strategy_std_msgs_Header),
  sizeof(SL_Bus_soccer_strategy_soccer_msgs_RobotGoal),
  sizeof(SL_Bus_soccer_strategy_geometry_msgs_Quaternion),
  sizeof(SL_Bus_soccer_strategy_geometry_msgs_Vector3),
  sizeof(SL_Bus_soccer_strategy_sensor_msgs_Imu),
  sizeof(SL_Bus_soccer_strategy_team_communication_robotInfo),
  sizeof(SL_Bus_soccer_strategy_team_communication_teamInfo),
  sizeof(SL_Bus_soccer_strategy_team_communication_game_state),
  sizeof(dsp_simulink_MovingAverage_soccer_strategy_T),
  sizeof(dsp_private_SlidingWindowAverageCG_soccer_strategy_T),
  sizeof(ExampleHelperSimulationRateControl_soccer_strategy_T),
  sizeof(robotics_slros_internal_block_Publisher_soccer_strategy_T),
  sizeof(robotics_slros_internal_block_Subscriber_soccer_strategy_T)
};

// data type name table
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T",
  "SL_Bus_ROSVariableLengthArrayInfo",
  "SL_Bus_soccer_strategy_ros_time_Time",
  "SL_Bus_soccer_strategy_std_msgs_Header",
  "SL_Bus_soccer_strategy_soccer_msgs_RobotGoal",
  "SL_Bus_soccer_strategy_geometry_msgs_Quaternion",
  "SL_Bus_soccer_strategy_geometry_msgs_Vector3",
  "SL_Bus_soccer_strategy_sensor_msgs_Imu",
  "SL_Bus_soccer_strategy_team_communication_robotInfo",
  "SL_Bus_soccer_strategy_team_communication_teamInfo",
  "SL_Bus_soccer_strategy_team_communication_game_state",
  "dsp_simulink_MovingAverage_soccer_strategy_T",
  "dsp_private_SlidingWindowAverageCG_soccer_strategy_T",
  "ExampleHelperSimulationRateControl_soccer_strategy_T",
  "robotics_slros_internal_block_Publisher_soccer_strategy_T",
  "robotics_slros_internal_block_Subscriber_soccer_strategy_T"
};

// data type transitions for block I/O structure
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&soccer_strategy_B.In1), 20, 0, 1 },

  { (char_T *)(&soccer_strategy_B.Constant3), 0, 0, 40 },

  { (char_T *)(&soccer_strategy_B.DiscreteTimeIntegrator), 1, 0, 23 },

  { (char_T *)(&soccer_strategy_B.MovingAverage5.MovingAverage), 0, 0, 1 },

  { (char_T *)(&soccer_strategy_B.MovingAverage4.MovingAverage), 0, 0, 1 },

  { (char_T *)(&soccer_strategy_B.MovingAverage3.MovingAverage), 0, 0, 1 },

  { (char_T *)(&soccer_strategy_B.MovingAverage1.MovingAverage), 0, 0, 1 },

  { (char_T *)(&soccer_strategy_B.MovingAverage2.MovingAverage), 0, 0, 1 },

  { (char_T *)(&soccer_strategy_B.MovingAverage.MovingAverage), 0, 0, 1 }
  ,

  { (char_T *)(&soccer_strategy_DW.obj), 27, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.obj_e), 28, 0, 2 },

  { (char_T *)(&soccer_strategy_DW.Delay_DSTATE), 0, 0, 5 },

  { (char_T *)(&soccer_strategy_DW.Trajectories_PWORK.LoggedData), 11, 0, 13 },

  { (char_T *)(&soccer_strategy_DW.Integrator_DSTATE), 1, 0, 9 },

  { (char_T *)(&soccer_strategy_DW.sfEvent), 6, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.obj_d), 26, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.FromWorkspace_IWORK.PrevIndex), 10, 0, 6 },

  { (char_T *)(&soccer_strategy_DW.temporalCounter_i1), 5, 0, 2 },

  { (char_T *)(&soccer_strategy_DW.EnabledSubsystem_SubsysRanBC), 2, 0, 11 },

  { (char_T *)(&soccer_strategy_DW.is_active_c3_soccer_strategy), 3, 0, 3 },

  { (char_T *)(&soccer_strategy_DW.objisempty), 8, 0, 4 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage5.gobj_0), 25, 0, 2 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage5.obj), 24, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage5.objisempty), 8, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage4.gobj_0), 25, 0, 2 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage4.obj), 24, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage4.objisempty), 8, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage3.gobj_0), 25, 0, 2 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage3.obj), 24, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage3.objisempty), 8, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage1.gobj_0), 25, 0, 2 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage1.obj), 24, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage1.objisempty), 8, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage2.gobj_0), 25, 0, 2 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage2.obj), 24, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage2.objisempty), 8, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage.gobj_0), 25, 0, 2 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage.obj), 24, 0, 1 },

  { (char_T *)(&soccer_strategy_DW.MovingAverage.objisempty), 8, 0, 1 }
};

// data type transition table for block I/O structure
static DataTypeTransitionTable rtBTransTable = {
  39U,
  rtBTransitions
};

// data type transitions for Parameters structure
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&soccer_strategy_P.ready[0]), 0, 0, 40 },

  { (char_T *)(&soccer_strategy_P.motorCalibration[0]), 1, 0, 36 },

  { (char_T *)(&soccer_strategy_P.Out1_Y0), 23, 0, 1 },

  { (char_T *)(&soccer_strategy_P.Constant_Value), 23, 0, 1 },

  { (char_T *)(&soccer_strategy_P.Out1_Y0_l), 20, 0, 1 },

  { (char_T *)(&soccer_strategy_P.Constant_Value_l), 20, 0, 1 },

  { (char_T *)(&soccer_strategy_P.Constant_Value_c), 17, 0, 1 },

  { (char_T *)(&soccer_strategy_P.Gain_Gain), 0, 0, 67 },

  { (char_T *)(&soccer_strategy_P.Constant_Value_o), 1, 0, 45 },

  { (char_T *)(&soccer_strategy_P.Delay_DelayLength), 7, 0, 5 }
};

// data type transition table for Parameters structure
static DataTypeTransitionTable rtPTransTable = {
  10U,
  rtPTransitions
};

// [EOF] soccer_strategy_dt.h
