//
//  soccerbot_dt.h
//
//  Sponsored License - for use in support of a program or activity
//  sponsored by MathWorks.  Not for government, commercial or other
//  non-sponsored organizational use.
//
//  Code generation for model "soccerbot".
//
//  Model version              : 1.19
//  Simulink Coder version : 8.13 (R2017b) 24-Jul-2017
//  C++ source code generated on : Sun Jun  3 00:59:48 2018
//
//  Target selection: ert.tlc
//  Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
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
  sizeof(SL_Bus_soccerbot_geometry_msgs_Point),
  sizeof(SL_Bus_ROSVariableLengthArrayInfo),
  sizeof(SL_Bus_soccerbot_ros_time_Time),
  sizeof(SL_Bus_soccerbot_std_msgs_Header),
  sizeof(SL_Bus_soccerbot_sensor_msgs_CompressedImage),
  sizeof(ExampleHelperSimulationRateCo_T),
  sizeof(robotics_slros_internal_blo_b_T),
  sizeof(robotics_slros_internal_block_T)
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
  "SL_Bus_soccerbot_geometry_msgs_Point",
  "SL_Bus_ROSVariableLengthArrayInfo",
  "SL_Bus_soccerbot_ros_time_Time",
  "SL_Bus_soccerbot_std_msgs_Header",
  "SL_Bus_soccerbot_sensor_msgs_CompressedImage",
  "ExampleHelperSimulationRateCo_T",
  "robotics_slros_internal_blo_b_T",
  "robotics_slros_internal_block_T"
};

// data type transitions for block I/O structure
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&soccerbot_B.Gain), 0, 0, 2 }
  ,

  { (char_T *)(&soccerbot_DW.obj), 20, 0, 1 },

  { (char_T *)(&soccerbot_DW.Scope_PWORK.LoggedData[0]), 11, 0, 2 },

  { (char_T *)(&soccerbot_DW.obj_d), 19, 0, 1 },

  { (char_T *)(&soccerbot_DW.obj_e), 21, 0, 1 },

  { (char_T *)(&soccerbot_DW.EnabledSubsystem_SubsysRanBC), 2, 0, 2 }
};

// data type transition table for block I/O structure
static DataTypeTransitionTable rtBTransTable = {
  6U,
  rtBTransitions
};

// data type transitions for Parameters structure
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&soccerbot_P.ctrlParam1), 0, 0, 2 },

  { (char_T *)(&soccerbot_P.Out1_Y0), 18, 0, 1 },

  { (char_T *)(&soccerbot_P.Constant_Value), 18, 0, 1 },

  { (char_T *)(&soccerbot_P.dataOut_Y0), 3, 0, 1 }
};

// data type transition table for Parameters structure
static DataTypeTransitionTable rtPTransTable = {
  4U,
  rtPTransitions
};

// [EOF] soccerbot_dt.h
