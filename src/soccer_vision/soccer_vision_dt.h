//
//  soccer_vision_dt.h
//
//  Sponsored License - for use in support of a program or activity
//  sponsored by MathWorks.  Not for government, commercial or other
//  non-sponsored organizational use.
//
//  Code generation for model "soccer_vision".
//
<<<<<<< HEAD
//  Model version              : 1.759
//  Simulink Coder version : 8.14 (R2018a) 06-Feb-2018
//  C++ source code generated on : Sat Jul 14 20:07:55 2018
//
//  Target selection: ert.tlc
//  Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
=======
//  Model version              : 1.770
//  Simulink Coder version : 8.14 (R2018a) 06-Feb-2018
//  C++ source code generated on : Sun Jul 15 23:50:31 2018
//
//  Target selection: ert.tlc
//  Embedded hardware selection: ARM Compatible->ARM 64-bit (LP64)
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
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
<<<<<<< HEAD
=======
  sizeof(SL_Bus_ROSVariableLengthArrayInfo),
  sizeof(SL_Bus_soccer_vision_ros_time_Time),
  sizeof(SL_Bus_soccer_vision_std_msgs_Header),
  sizeof(SL_Bus_soccer_vision_sensor_msgs_Image),
  sizeof(robotics_slros_internal_block_ReadImage_soccer_vision_T),
  sizeof(robotics_slros_internal_block_Subscriber_soccer_vision_T),
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  sizeof(int16_T),
  sizeof(int32_T),
  sizeof(int32_T),
  sizeof(int32_T),
  sizeof(map2Dto3D_soccer_vision_T),
  sizeof(int32_T),
  sizeof(int16_T),
  sizeof(int32_T)
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
<<<<<<< HEAD
=======
  "SL_Bus_ROSVariableLengthArrayInfo",
  "SL_Bus_soccer_vision_ros_time_Time",
  "SL_Bus_soccer_vision_std_msgs_Header",
  "SL_Bus_soccer_vision_sensor_msgs_Image",
  "robotics_slros_internal_block_ReadImage_soccer_vision_T",
  "robotics_slros_internal_block_Subscriber_soccer_vision_T",
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  "int16_T",
  "int32_T",
  "int32_T",
  "int32_T",
  "map2Dto3D_soccer_vision_T",
  "int32_T",
  "int16_T",
  "int32_T"
};

// data type transitions for block I/O structure
static DataTypeTransition rtBTransitions[] = {
<<<<<<< HEAD
  { (char_T *)(&soccer_vision_B.HoughTransform_o1[0]), 0, 0, 71820 },

  { (char_T *)(&soccer_vision_B.HoughTransform_o3[0]), 0, 0, 399 },

  { (char_T *)(&soccer_vision_B.HoughTransform_o2[0]), 0, 0, 180 },

  { (char_T *)(&soccer_vision_B.rho[0]), 0, 0, 40 },

  { (char_T *)(&soccer_vision_B.TmpSignalConversionAtCoverAboveHorizonInport1[0]),
    3, 0, 326400 },

  { (char_T *)(&soccer_vision_B.EdgeDetection[0]), 8, 0, 19200 },

  { (char_T *)(&soccer_vision_B.HoughLines[0]), 6, 0, 36 }
  ,

  { (char_T *)(&soccer_vision_DW.FindLocalMaxima_TEMP_IN_DWORKS[0]), 0, 0, 71820
  },

=======
  { (char_T *)(&soccer_vision_B.Resize[0]), 3, 0, 19200 },

  { (char_T *)(&soccer_vision_B.EdgeDetection[0]), 8, 0, 19200 },

  { (char_T *)(&soccer_vision_B.In1), 17, 0, 1 },

  { (char_T *)(&soccer_vision_B.HoughTransform_o1[0]), 0, 0, 72399 },

  { (char_T *)(&soccer_vision_B.rho[0]), 0, 0, 40 },

  { (char_T *)(&soccer_vision_B.thetaField[0]), 0, 0, 24 },

  { (char_T *)(&soccer_vision_B.HoughLines[0]), 6, 0, 36 },

  { (char_T *)(&soccer_vision_B.ReadImage_o2), 3, 0, 1 }
  ,

  { (char_T *)(&soccer_vision_DW.obj), 18, 0, 1 },

  { (char_T *)(&soccer_vision_DW.FindLocalMaxima_TEMP_IN_DWORKS[0]), 0, 0, 71820
  },

  { (char_T *)(&soccer_vision_DW.obj_h), 19, 0, 1 },

>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  { (char_T *)(&soccer_vision_DW.Memory_PreviousInput[0]), 0, 0, 18 },

  { (char_T *)(&soccer_vision_DW.EdgeDetection_VO_DW[0]), 6, 0, 108 },

  { (char_T *)(&soccer_vision_DW.state[0]), 7, 0, 625 },

<<<<<<< HEAD
  { (char_T *)(&soccer_vision_DW.EdgeDetection_MEAN_FACTOR_DW), 16, 0, 1 }
=======
  { (char_T *)(&soccer_vision_DW.EdgeDetection_MEAN_FACTOR_DW), 22, 0, 1 },

  { (char_T *)(&soccer_vision_DW.EnabledSubsystem_SubsysRanBC), 2, 0, 1 },

  { (char_T *)(&soccer_vision_DW.ColorSpaceConversion_DWORK1[0]), 3, 0, 921600 }
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
};

// data type transition table for block I/O structure
static DataTypeTransitionTable rtBTransTable = {
<<<<<<< HEAD
  12U,
=======
  17U,
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  rtBTransitions
};

// data type transitions for Parameters structure
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&soccer_vision_P.clusterProximityThreshold), 0, 0, 5 },

<<<<<<< HEAD
  { (char_T *)(&soccer_vision_P.EdgeDetection_thresholdTuning), 15, 0, 1 },

  { (char_T *)(&soccer_vision_P.Constant6_Value), 0, 0, 24 },

  { (char_T *)(&soccer_vision_P.CoverAboveHorizon_RTP_OPACITY), 14, 0, 1 }
=======
  { (char_T *)(&soccer_vision_P.EdgeDetection_thresholdTuning), 21, 0, 1 },

  { (char_T *)(&soccer_vision_P.Out1_Y0), 17, 0, 1 },

  { (char_T *)(&soccer_vision_P.Constant_Value), 17, 0, 1 },

  { (char_T *)(&soccer_vision_P.Constant6_Value), 0, 0, 24 },

  { (char_T *)(&soccer_vision_P.CoverAboveHorizon_RTP_OPACITY), 20, 0, 1 }
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
};

// data type transition table for Parameters structure
static DataTypeTransitionTable rtPTransTable = {
<<<<<<< HEAD
  4U,
=======
  6U,
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
  rtPTransitions
};

// [EOF] soccer_vision_dt.h
