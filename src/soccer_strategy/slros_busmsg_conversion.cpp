#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_soccer_strategy_geometry_msgs_Quaternion and geometry_msgs::Quaternion

void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_soccer_strategy_geometry_msgs_Quaternion const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr->w =  busPtr->W;
  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_soccer_strategy_geometry_msgs_Quaternion* busPtr, geometry_msgs::Quaternion const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  busPtr->W =  msgPtr->w;
  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_soccer_strategy_geometry_msgs_Vector3 and geometry_msgs::Vector3

void convertFromBus(geometry_msgs::Vector3* msgPtr, SL_Bus_soccer_strategy_geometry_msgs_Vector3 const* busPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  msgPtr->x =  busPtr->X;
  msgPtr->y =  busPtr->Y;
  msgPtr->z =  busPtr->Z;
}

void convertToBus(SL_Bus_soccer_strategy_geometry_msgs_Vector3* busPtr, geometry_msgs::Vector3 const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  busPtr->X =  msgPtr->x;
  busPtr->Y =  msgPtr->y;
  busPtr->Z =  msgPtr->z;
}


// Conversions between SL_Bus_soccer_strategy_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_soccer_strategy_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_soccer_strategy_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_soccer_strategy_sensor_msgs_Imu and sensor_msgs::Imu

void convertFromBus(sensor_msgs::Imu* msgPtr, SL_Bus_soccer_strategy_sensor_msgs_Imu const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/Imu");

  convertFromBus(&msgPtr->angular_velocity, &busPtr->AngularVelocity);
  convertFromBusFixedPrimitiveArray(msgPtr->angular_velocity_covariance, busPtr->AngularVelocityCovariance);
  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBus(&msgPtr->linear_acceleration, &busPtr->LinearAcceleration);
  convertFromBusFixedPrimitiveArray(msgPtr->linear_acceleration_covariance, busPtr->LinearAccelerationCovariance);
  convertFromBus(&msgPtr->orientation, &busPtr->Orientation);
  convertFromBusFixedPrimitiveArray(msgPtr->orientation_covariance, busPtr->OrientationCovariance);
}

void convertToBus(SL_Bus_soccer_strategy_sensor_msgs_Imu* busPtr, sensor_msgs::Imu const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/Imu");

  convertToBus(&busPtr->AngularVelocity, &msgPtr->angular_velocity);
  convertToBusFixedPrimitiveArray(busPtr->AngularVelocityCovariance, msgPtr->angular_velocity_covariance, slros::NoopWarning());
  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBus(&busPtr->LinearAcceleration, &msgPtr->linear_acceleration);
  convertToBusFixedPrimitiveArray(busPtr->LinearAccelerationCovariance, msgPtr->linear_acceleration_covariance, slros::NoopWarning());
  convertToBus(&busPtr->Orientation, &msgPtr->orientation);
  convertToBusFixedPrimitiveArray(busPtr->OrientationCovariance, msgPtr->orientation_covariance, slros::NoopWarning());
}


// Conversions between SL_Bus_soccer_strategy_soccer_msgs_RobotGoal and soccer_msgs::RobotGoal

void convertFromBus(soccer_msgs::RobotGoal* msgPtr, SL_Bus_soccer_strategy_soccer_msgs_RobotGoal const* busPtr)
{
  const std::string rosMessageType("soccer_msgs/RobotGoal");

  convertFromBus(&msgPtr->header, &busPtr->Header);
  convertFromBusFixedPrimitiveArray(msgPtr->trajectories, busPtr->Trajectories);
}

void convertToBus(SL_Bus_soccer_strategy_soccer_msgs_RobotGoal* busPtr, soccer_msgs::RobotGoal const* msgPtr)
{
  const std::string rosMessageType("soccer_msgs/RobotGoal");

  convertToBus(&busPtr->Header, &msgPtr->header);
  convertToBusFixedPrimitiveArray(busPtr->Trajectories, msgPtr->trajectories, slros::NoopWarning());
}


// Conversions between SL_Bus_soccer_strategy_std_msgs_Header and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_soccer_strategy_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_soccer_strategy_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}


// Conversions between SL_Bus_soccer_strategy_team_communication_game_state and team_communication::game_state

void convertFromBus(team_communication::game_state* msgPtr, SL_Bus_soccer_strategy_team_communication_game_state const* busPtr)
{
  const std::string rosMessageType("team_communication/game_state");

  msgPtr->dropInTeam =  busPtr->DropInTeam;
  msgPtr->dropInTime =  busPtr->DropInTime;
  msgPtr->firsthalf =  busPtr->Firsthalf;
  msgPtr->gameType =  busPtr->GameType;
  convertFromBusFixedPrimitiveArray(msgPtr->header, busPtr->Header);
  msgPtr->kickoffTeam =  busPtr->KickoffTeam;
  msgPtr->packetNum =  busPtr->PacketNum;
  msgPtr->playersPerTeam =  busPtr->PlayersPerTeam;
  msgPtr->protocol_version =  busPtr->ProtocolVersion;
  msgPtr->secondaryState =  busPtr->SecondaryState;
  convertFromBusFixedPrimitiveArray(msgPtr->secondaryStateInfo, busPtr->SecondaryStateInfo);
  msgPtr->secondaryTime =  busPtr->SecondaryTime;
  msgPtr->secsRemaining =  busPtr->SecsRemaining;
  msgPtr->state =  busPtr->State;
  convertFromBusFixedNestedArray(msgPtr->Teams, busPtr->Teams);
}

void convertToBus(SL_Bus_soccer_strategy_team_communication_game_state* busPtr, team_communication::game_state const* msgPtr)
{
  const std::string rosMessageType("team_communication/game_state");

  busPtr->DropInTeam =  msgPtr->dropInTeam;
  busPtr->DropInTime =  msgPtr->dropInTime;
  busPtr->Firsthalf =  msgPtr->firsthalf;
  busPtr->GameType =  msgPtr->gameType;
  convertToBusFixedPrimitiveArray(busPtr->Header, msgPtr->header, slros::NoopWarning());
  busPtr->KickoffTeam =  msgPtr->kickoffTeam;
  busPtr->PacketNum =  msgPtr->packetNum;
  busPtr->PlayersPerTeam =  msgPtr->playersPerTeam;
  busPtr->ProtocolVersion =  msgPtr->protocol_version;
  busPtr->SecondaryState =  msgPtr->secondaryState;
  convertToBusFixedPrimitiveArray(busPtr->SecondaryStateInfo, msgPtr->secondaryStateInfo, slros::NoopWarning());
  busPtr->SecondaryTime =  msgPtr->secondaryTime;
  busPtr->SecsRemaining =  msgPtr->secsRemaining;
  busPtr->State =  msgPtr->state;
  convertToBusFixedNestedArray(busPtr->Teams, msgPtr->Teams, slros::NoopWarning());
}


// Conversions between SL_Bus_soccer_strategy_team_communication_robotInfo and team_communication::robotInfo

void convertFromBus(team_communication::robotInfo* msgPtr, SL_Bus_soccer_strategy_team_communication_robotInfo const* busPtr)
{
  const std::string rosMessageType("team_communication/robotInfo");

  msgPtr->penalty =  busPtr->Penalty;
  msgPtr->redCardCount =  busPtr->RedCardCount;
  msgPtr->secsTillUnpenalised =  busPtr->SecsTillUnpenalised;
  msgPtr->yellowCardCount =  busPtr->YellowCardCount;
}

void convertToBus(SL_Bus_soccer_strategy_team_communication_robotInfo* busPtr, team_communication::robotInfo const* msgPtr)
{
  const std::string rosMessageType("team_communication/robotInfo");

  busPtr->Penalty =  msgPtr->penalty;
  busPtr->RedCardCount =  msgPtr->redCardCount;
  busPtr->SecsTillUnpenalised =  msgPtr->secsTillUnpenalised;
  busPtr->YellowCardCount =  msgPtr->yellowCardCount;
}


// Conversions between SL_Bus_soccer_strategy_team_communication_teamInfo and team_communication::teamInfo

void convertFromBus(team_communication::teamInfo* msgPtr, SL_Bus_soccer_strategy_team_communication_teamInfo const* busPtr)
{
  const std::string rosMessageType("team_communication/teamInfo");

  convertFromBus(&msgPtr->coach, &busPtr->Coach);
  convertFromBusFixedPrimitiveArray(msgPtr->coachMessage, busPtr->CoachMessage);
  msgPtr->coachSequence =  busPtr->CoachSequence;
  msgPtr->penaltyShot =  busPtr->PenaltyShot;
  convertFromBusFixedNestedArray(msgPtr->players, busPtr->Players);
  msgPtr->score =  busPtr->Score;
  msgPtr->singleShots =  busPtr->SingleShots;
  msgPtr->teamColour =  busPtr->TeamColour;
  msgPtr->teamNum =  busPtr->TeamNum;
}

void convertToBus(SL_Bus_soccer_strategy_team_communication_teamInfo* busPtr, team_communication::teamInfo const* msgPtr)
{
  const std::string rosMessageType("team_communication/teamInfo");

  convertToBus(&busPtr->Coach, &msgPtr->coach);
  convertToBusFixedPrimitiveArray(busPtr->CoachMessage, msgPtr->coachMessage, slros::NoopWarning());
  busPtr->CoachSequence =  msgPtr->coachSequence;
  busPtr->PenaltyShot =  msgPtr->penaltyShot;
  convertToBusFixedNestedArray(busPtr->Players, msgPtr->players, slros::NoopWarning());
  busPtr->Score =  msgPtr->score;
  busPtr->SingleShots =  msgPtr->singleShots;
  busPtr->TeamColour =  msgPtr->teamColour;
  busPtr->TeamNum =  msgPtr->teamNum;
}

