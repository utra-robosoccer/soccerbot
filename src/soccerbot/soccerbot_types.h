//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: soccerbot_types.h
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
#ifndef RTW_HEADER_soccerbot_types_h_
#define RTW_HEADER_soccerbot_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccerbot_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccerbot_geometry_msgs_Point_

// MsgType=geometry_msgs/Point
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_soccerbot_geometry_msgs_Point;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccerbot_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccerbot_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_soccerbot_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccerbot_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccerbot_std_msgs_Header_

// MsgType=std_msgs/Header
typedef struct {
  uint32_T Seq;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;

  // MsgType=ros_time/Time
  SL_Bus_soccerbot_ros_time_Time Stamp;
} SL_Bus_soccerbot_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccerbot_sensor_msgs_CompressedImage_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccerbot_sensor_msgs_CompressedImage_

// MsgType=sensor_msgs/CompressedImage
typedef struct {
  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=Format_SL_Info:TruncateAction=warn 
  uint8_T Format[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Format
  SL_Bus_ROSVariableLengthArrayInfo Format_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Data_SL_Info:TruncateAction=warn
  uint8_T Data[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Data
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;

  // MsgType=std_msgs/Header
  SL_Bus_soccerbot_std_msgs_Header Header;
} SL_Bus_soccerbot_sensor_msgs_CompressedImage;

#endif

#ifndef typedef_ExampleHelperSimulationRateCo_T
#define typedef_ExampleHelperSimulationRateCo_T

typedef struct {
  int32_T isInitialized;
} ExampleHelperSimulationRateCo_T;

#endif                                 //typedef_ExampleHelperSimulationRateCo_T

#ifndef typedef_robotics_slros_internal_block_T
#define typedef_robotics_slros_internal_block_T

typedef struct {
  int32_T isInitialized;
} robotics_slros_internal_block_T;

#endif                                 //typedef_robotics_slros_internal_block_T

#ifndef struct_tag_sThDE1hV6yrG4ak7SBty8CB
#define struct_tag_sThDE1hV6yrG4ak7SBty8CB

struct tag_sThDE1hV6yrG4ak7SBty8CB
{
  char_T Name[4];
  char_T DataType[5];
  real_T NumChannels;
  real_T ChannelOrder[3];
  boolean_T HasAlpha;
  boolean_T IsBayer;
};

#endif                                 //struct_tag_sThDE1hV6yrG4ak7SBty8CB

#ifndef typedef_sThDE1hV6yrG4ak7SBty8CB_socce_T
#define typedef_sThDE1hV6yrG4ak7SBty8CB_socce_T

typedef struct tag_sThDE1hV6yrG4ak7SBty8CB sThDE1hV6yrG4ak7SBty8CB_socce_T;

#endif                                 //typedef_sThDE1hV6yrG4ak7SBty8CB_socce_T

#ifndef typedef_cell_wrap_soccerbot_T
#define typedef_cell_wrap_soccerbot_T

typedef struct {
  uint32_T f1[8];
} cell_wrap_soccerbot_T;

#endif                                 //typedef_cell_wrap_soccerbot_T

#ifndef typedef_robotics_slros_internal_blo_b_T
#define typedef_robotics_slros_internal_blo_b_T

typedef struct {
  int32_T isInitialized;
  cell_wrap_soccerbot_T inputVarSize;
  uint8_T Image[230400];
  uint32_T ImageSize[2];
} robotics_slros_internal_blo_b_T;

#endif                                 //typedef_robotics_slros_internal_blo_b_T

// Parameters (auto storage)
typedef struct P_soccerbot_T_ P_soccerbot_T;

// Forward declaration for rtModel
typedef struct tag_RTM_soccerbot_T RT_MODEL_soccerbot_T;

#endif                                 // RTW_HEADER_soccerbot_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
