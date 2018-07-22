//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: soccer_strategy_types.h
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
#ifndef RTW_HEADER_soccer_strategy_types_h_
#define RTW_HEADER_soccer_strategy_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

typedef struct {
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
} SL_Bus_ROSVariableLengthArrayInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_ros_time_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_ros_time_Time_

// MsgType=ros_time/Time
typedef struct {
  real_T Sec;
  real_T Nsec;
} SL_Bus_soccer_strategy_ros_time_Time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_std_msgs_Header_

// MsgType=std_msgs/Header
typedef struct {
  uint32_T Seq;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=FrameId_SL_Info:TruncateAction=warn 
  uint8_T FrameId[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=FrameId
  SL_Bus_ROSVariableLengthArrayInfo FrameId_SL_Info;

  // MsgType=ros_time/Time
  SL_Bus_soccer_strategy_ros_time_Time Stamp;
} SL_Bus_soccer_strategy_std_msgs_Header;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_soccer_msgs_RobotGoal_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_soccer_msgs_RobotGoal_

// MsgType=soccer_msgs/RobotGoal
typedef struct {
  real32_T Trajectories[20];

  // MsgType=std_msgs/Header
  SL_Bus_soccer_strategy_std_msgs_Header Header;
} SL_Bus_soccer_strategy_soccer_msgs_RobotGoal;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_geometry_msgs_Quaternion_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_geometry_msgs_Quaternion_

// MsgType=geometry_msgs/Quaternion
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
  real_T W;
} SL_Bus_soccer_strategy_geometry_msgs_Quaternion;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_geometry_msgs_Vector3_

// MsgType=geometry_msgs/Vector3
typedef struct {
  real_T X;
  real_T Y;
  real_T Z;
} SL_Bus_soccer_strategy_geometry_msgs_Vector3;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_sensor_msgs_Imu_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_sensor_msgs_Imu_

// MsgType=sensor_msgs/Imu
typedef struct {
  real_T OrientationCovariance[9];
  real_T AngularVelocityCovariance[9];
  real_T LinearAccelerationCovariance[9];

  // MsgType=std_msgs/Header
  SL_Bus_soccer_strategy_std_msgs_Header Header;

  // MsgType=geometry_msgs/Quaternion
  SL_Bus_soccer_strategy_geometry_msgs_Quaternion Orientation;

  // MsgType=geometry_msgs/Vector3
  SL_Bus_soccer_strategy_geometry_msgs_Vector3 AngularVelocity;

  // MsgType=geometry_msgs/Vector3
  SL_Bus_soccer_strategy_geometry_msgs_Vector3 LinearAcceleration;
} SL_Bus_soccer_strategy_sensor_msgs_Imu;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_team_communication_robotInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_team_communication_robotInfo_

// MsgType=team_communication/robotInfo
typedef struct {
  uint8_T Penalty;
  uint8_T SecsTillUnpenalised;
  uint8_T YellowCardCount;
  uint8_T RedCardCount;
} SL_Bus_soccer_strategy_team_communication_robotInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_team_communication_teamInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_team_communication_teamInfo_

// MsgType=team_communication/teamInfo
typedef struct {
  uint8_T TeamNum;
  uint8_T TeamColour;
  uint8_T Score;
  uint8_T PenaltyShot;
  uint16_T SingleShots;
  uint8_T CoachSequence;
  uint8_T CoachMessage[253];

  // MsgType=team_communication/robotInfo
  SL_Bus_soccer_strategy_team_communication_robotInfo Coach;

  // MsgType=team_communication/robotInfo
  SL_Bus_soccer_strategy_team_communication_robotInfo Players[11];
} SL_Bus_soccer_strategy_team_communication_teamInfo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_team_communication_game_state_
#define DEFINED_TYPEDEF_FOR_SL_Bus_soccer_strategy_team_communication_game_state_

// MsgType=team_communication/game_state
typedef struct {
  uint16_T ProtocolVersion;
  uint8_T PacketNum;
  uint8_T PlayersPerTeam;
  uint8_T GameType;
  uint8_T State;
  uint8_T Firsthalf;
  uint8_T KickoffTeam;
  uint8_T SecondaryState;
  uint8_T DropInTeam;
  uint16_T DropInTime;
  uint16_T SecsRemaining;
  uint16_T SecondaryTime;
  uint8_T Header[4];
  uint8_T SecondaryStateInfo[4];

  // MsgType=team_communication/teamInfo
  SL_Bus_soccer_strategy_team_communication_teamInfo Teams[2];
} SL_Bus_soccer_strategy_team_communication_game_state;

#endif

#ifndef typedef_dsp_private_SlidingWindowAverageCG_soccer_strategy_T
#define typedef_dsp_private_SlidingWindowAverageCG_soccer_strategy_T

typedef struct {
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T pCumSum;
  real_T pCumSumRev[59];
  real_T pCumRevIndex;
} dsp_private_SlidingWindowAverageCG_soccer_strategy_T;

#endif                                 //typedef_dsp_private_SlidingWindowAverageCG_soccer_strategy_T

#ifndef typedef_cell_wrap_soccer_strategy_T
#define typedef_cell_wrap_soccer_strategy_T

typedef struct {
  uint32_T f1[8];
} cell_wrap_soccer_strategy_T;

#endif                                 //typedef_cell_wrap_soccer_strategy_T

#ifndef typedef_dsp_simulink_MovingAverage_soccer_strategy_T
#define typedef_dsp_simulink_MovingAverage_soccer_strategy_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T TunablePropsChanged;
  cell_wrap_soccer_strategy_T inputVarSize;
  dsp_private_SlidingWindowAverageCG_soccer_strategy_T *pStatistic;
  int32_T NumChannels;
} dsp_simulink_MovingAverage_soccer_strategy_T;

#endif                                 //typedef_dsp_simulink_MovingAverage_soccer_strategy_T

#ifndef typedef_ExampleHelperSimulationRateControl_soccer_strategy_T
#define typedef_ExampleHelperSimulationRateControl_soccer_strategy_T

typedef struct {
  int32_T isInitialized;
} ExampleHelperSimulationRateControl_soccer_strategy_T;

#endif                                 //typedef_ExampleHelperSimulationRateControl_soccer_strategy_T

#ifndef typedef_robotics_slros_internal_block_Publisher_soccer_strategy_T
#define typedef_robotics_slros_internal_block_Publisher_soccer_strategy_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_block_Publisher_soccer_strategy_T;

#endif                                 //typedef_robotics_slros_internal_block_Publisher_soccer_strategy_T

#ifndef typedef_robotics_slros_internal_block_Subscriber_soccer_strategy_T
#define typedef_robotics_slros_internal_block_Subscriber_soccer_strategy_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
} robotics_slros_internal_block_Subscriber_soccer_strategy_T;

#endif                                 //typedef_robotics_slros_internal_block_Subscriber_soccer_strategy_T

// Parameters (default storage)
typedef struct P_soccer_strategy_T_ P_soccer_strategy_T;

// Forward declaration for rtModel
typedef struct tag_RTM_soccer_strategy_T RT_MODEL_soccer_strategy_T;

#endif                                 // RTW_HEADER_soccer_strategy_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
