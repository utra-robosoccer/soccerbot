//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: soccer_vision_types.h
//
// Code generated for Simulink model 'soccer_vision'.
//
// Model version                  : 1.770
// Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
// C/C++ source code generated on : Sun Jul 15 23:50:31 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 64-bit (LP64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_soccer_vision_types_h_
#define RTW_HEADER_soccer_vision_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccer_vision_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccer_vision_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_soccer_vision_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccer_vision_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccer_vision_std_msgs_Header_

// MsgType=std_msgs/Header
typedef struct {
  uint32_T Seq;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;

  // MsgType=ros_time/Time
  SL_Bus_soccer_vision_ros_time_Time Stamp;
} SL_Bus_soccer_vision_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccer_vision_sensor_msgs_Image_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccer_vision_sensor_msgs_Image_

// MsgType=sensor_msgs/Image
typedef struct {
  uint32_T Height;
  uint32_T Width;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=Encoding_SL_Info:TruncateAction=warn 
  uint8_T Encoding[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Encoding
  SL_Bus_ROSVariableLengthArrayInfo Encoding_SL_Info;
  uint8_T IsBigendian;
  uint32_T Step;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=Data_SL_Info:TruncateAction=warn
  uint8_T Data[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=Data
  SL_Bus_ROSVariableLengthArrayInfo Data_SL_Info;

  // MsgType=std_msgs/Header
  SL_Bus_soccer_vision_std_msgs_Header Header;
} SL_Bus_soccer_vision_sensor_msgs_Image;

#endif

// Custom Type definition for MATLABSystem: '<S4>/Read Image'
#include <stddef.h>
#ifndef typedef_robotics_slros_internal_block_Subscriber_soccer_vision_T
#define typedef_robotics_slros_internal_block_Subscriber_soccer_vision_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_block_Subscriber_soccer_vision_T;

#endif                                 //typedef_robotics_slros_internal_block_Subscriber_soccer_vision_T

#ifndef typedef_map2Dto3D_soccer_vision_T
#define typedef_map2Dto3D_soccer_vision_T

typedef struct {
  int32_T isInitialized;
  real_T ticksUntilNextHit;
} map2Dto3D_soccer_vision_T;

#endif                                 //typedef_map2Dto3D_soccer_vision_T

#ifndef typedef_Line2f_soccer_vision_T
#define typedef_Line2f_soccer_vision_T

typedef struct {
  real_T rho;
  real_T theta;
} Line2f_soccer_vision_T;

#endif                                 //typedef_Line2f_soccer_vision_T

#ifndef typedef_Point2f_soccer_vision_T
#define typedef_Point2f_soccer_vision_T

typedef struct {
  real_T x;
  real_T y;
} Point2f_soccer_vision_T;

#endif                                 //typedef_Point2f_soccer_vision_T

// Custom Type definition for MATLAB Function: '<S2>/Filter'
#ifndef struct_tag_sYtAINW4VTWzUrJuBkN3Z7
#define struct_tag_sYtAINW4VTWzUrJuBkN3Z7

struct tag_sYtAINW4VTWzUrJuBkN3Z7
{
  uint32_T distance;
  uint32_T start;
  uint32_T replicates;
  uint32_T emptyaction;
  uint32_T onlinephase;
  uint32_T options;
  uint32_T maxiter;
  uint32_T display;
};

#endif                                 //struct_tag_sYtAINW4VTWzUrJuBkN3Z7

#ifndef typedef_sYtAINW4VTWzUrJuBkN3Z7_soccer_vision_T
#define typedef_sYtAINW4VTWzUrJuBkN3Z7_soccer_vision_T

typedef struct tag_sYtAINW4VTWzUrJuBkN3Z7 sYtAINW4VTWzUrJuBkN3Z7_soccer_vision_T;

#endif                                 //typedef_sYtAINW4VTWzUrJuBkN3Z7_soccer_vision_T

#ifndef struct_tag_spmDXMrbi73sp7C198PnRNF
#define struct_tag_spmDXMrbi73sp7C198PnRNF

struct tag_spmDXMrbi73sp7C198PnRNF
{
  uint32_T Display;
  uint32_T MaxFunEvals;
  uint32_T MaxIter;
  uint32_T TolBnd;
  uint32_T TolFun;
  uint32_T TolTypeFun;
  uint32_T TolX;
  uint32_T TolTypeX;
  uint32_T GradObj;
  uint32_T Jacobian;
  uint32_T DerivStep;
  uint32_T FunValCheck;
  uint32_T Robust;
  uint32_T RobustWgtFun;
  uint32_T WgtFun;
  uint32_T Tune;
  uint32_T UseParallel;
  uint32_T UseSubstreams;
  uint32_T OutputFcn;
};

#endif                                 //struct_tag_spmDXMrbi73sp7C198PnRNF

#ifndef typedef_spmDXMrbi73sp7C198PnRNF_soccer_vision_T
#define typedef_spmDXMrbi73sp7C198PnRNF_soccer_vision_T

typedef struct tag_spmDXMrbi73sp7C198PnRNF
  spmDXMrbi73sp7C198PnRNF_soccer_vision_T;

#endif                                 //typedef_spmDXMrbi73sp7C198PnRNF_soccer_vision_T

// Custom Type definition for MATLAB Function: '<S12>/Label Field Lines'
#ifndef struct_tag_spGKsvEVm7uA89hv31XX4LH
#define struct_tag_spGKsvEVm7uA89hv31XX4LH

struct tag_spGKsvEVm7uA89hv31XX4LH
{
  uint32_T MissingPlacement;
  uint32_T ComparisonMethod;
};

#endif                                 //struct_tag_spGKsvEVm7uA89hv31XX4LH

#ifndef typedef_spGKsvEVm7uA89hv31XX4LH_soccer_vision_T
#define typedef_spGKsvEVm7uA89hv31XX4LH_soccer_vision_T

typedef struct tag_spGKsvEVm7uA89hv31XX4LH
  spGKsvEVm7uA89hv31XX4LH_soccer_vision_T;

#endif                                 //typedef_spGKsvEVm7uA89hv31XX4LH_soccer_vision_T

// Custom Type definition for MATLAB Function: '<S2>/Filter'
#ifndef struct_tag_spLjAHi9hGjlscmJkpZ9eWC
#define struct_tag_spLjAHi9hGjlscmJkpZ9eWC

struct tag_spLjAHi9hGjlscmJkpZ9eWC
{
  real_T MaxIter;
  boolean_T UseParallel;
  boolean_T UseSubstreams;
};

#endif                                 //struct_tag_spLjAHi9hGjlscmJkpZ9eWC

#ifndef typedef_spLjAHi9hGjlscmJkpZ9eWC_soccer_vision_T
#define typedef_spLjAHi9hGjlscmJkpZ9eWC_soccer_vision_T

typedef struct tag_spLjAHi9hGjlscmJkpZ9eWC
  spLjAHi9hGjlscmJkpZ9eWC_soccer_vision_T;

#endif                                 //typedef_spLjAHi9hGjlscmJkpZ9eWC_soccer_vision_T

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

#ifndef typedef_sThDE1hV6yrG4ak7SBty8CB_soccer_vision_T
#define typedef_sThDE1hV6yrG4ak7SBty8CB_soccer_vision_T

typedef struct tag_sThDE1hV6yrG4ak7SBty8CB
  sThDE1hV6yrG4ak7SBty8CB_soccer_vision_T;

#endif                                 //typedef_sThDE1hV6yrG4ak7SBty8CB_soccer_vision_T

#ifndef typedef_cell_wrap_soccer_vision_T
#define typedef_cell_wrap_soccer_vision_T

typedef struct {
  uint32_T f1[8];
} cell_wrap_soccer_vision_T;

#endif                                 //typedef_cell_wrap_soccer_vision_T

// Custom Type definition for MATLAB Function: '<S12>/Label Field Lines'
#ifndef struct_tag_sJCxfmxS8gBOONUZjbjUd9E
#define struct_tag_sJCxfmxS8gBOONUZjbjUd9E

struct tag_sJCxfmxS8gBOONUZjbjUd9E
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  char_T PartialMatching[6];
  boolean_T IgnoreNulls;
};

#endif                                 //struct_tag_sJCxfmxS8gBOONUZjbjUd9E

#ifndef typedef_sJCxfmxS8gBOONUZjbjUd9E_soccer_vision_T
#define typedef_sJCxfmxS8gBOONUZjbjUd9E_soccer_vision_T

typedef struct tag_sJCxfmxS8gBOONUZjbjUd9E
  sJCxfmxS8gBOONUZjbjUd9E_soccer_vision_T;

#endif                                 //typedef_sJCxfmxS8gBOONUZjbjUd9E_soccer_vision_T

#ifndef typedef_robotics_slros_internal_block_ReadImage_soccer_vision_T
#define typedef_robotics_slros_internal_block_ReadImage_soccer_vision_T

typedef struct {
  int32_T isInitialized;
  cell_wrap_soccer_vision_T inputVarSize;
  uint8_T Image[921600];
  uint32_T ImageSize[2];
} robotics_slros_internal_block_ReadImage_soccer_vision_T;

#endif                                 //typedef_robotics_slros_internal_block_ReadImage_soccer_vision_T

// Parameters (default storage)
typedef struct P_soccer_vision_T_ P_soccer_vision_T;

// Forward declaration for rtModel
typedef struct tag_RTM_soccer_vision_T RT_MODEL_soccer_vision_T;

#endif                                 // RTW_HEADER_soccer_vision_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
