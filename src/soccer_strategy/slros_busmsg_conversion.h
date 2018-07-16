#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <soccer_msgs/RobotGoal.h>
#include <std_msgs/Header.h>
#include <team_communication/game_state.h>
#include <team_communication/robotInfo.h>
#include <team_communication/teamInfo.h>
#include "soccer_strategy_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Quaternion* msgPtr, SL_Bus_soccer_strategy_geometry_msgs_Quaternion const* busPtr);
void convertToBus(SL_Bus_soccer_strategy_geometry_msgs_Quaternion* busPtr, geometry_msgs::Quaternion const* msgPtr);

void convertFromBus(geometry_msgs::Vector3* msgPtr, SL_Bus_soccer_strategy_geometry_msgs_Vector3 const* busPtr);
void convertToBus(SL_Bus_soccer_strategy_geometry_msgs_Vector3* busPtr, geometry_msgs::Vector3 const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_soccer_strategy_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_soccer_strategy_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(sensor_msgs::Imu* msgPtr, SL_Bus_soccer_strategy_sensor_msgs_Imu const* busPtr);
void convertToBus(SL_Bus_soccer_strategy_sensor_msgs_Imu* busPtr, sensor_msgs::Imu const* msgPtr);

void convertFromBus(soccer_msgs::RobotGoal* msgPtr, SL_Bus_soccer_strategy_soccer_msgs_RobotGoal const* busPtr);
void convertToBus(SL_Bus_soccer_strategy_soccer_msgs_RobotGoal* busPtr, soccer_msgs::RobotGoal const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_soccer_strategy_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_soccer_strategy_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);

void convertFromBus(team_communication::game_state* msgPtr, SL_Bus_soccer_strategy_team_communication_game_state const* busPtr);
void convertToBus(SL_Bus_soccer_strategy_team_communication_game_state* busPtr, team_communication::game_state const* msgPtr);

void convertFromBus(team_communication::robotInfo* msgPtr, SL_Bus_soccer_strategy_team_communication_robotInfo const* busPtr);
void convertToBus(SL_Bus_soccer_strategy_team_communication_robotInfo* busPtr, team_communication::robotInfo const* msgPtr);

void convertFromBus(team_communication::teamInfo* msgPtr, SL_Bus_soccer_strategy_team_communication_teamInfo const* busPtr);
void convertToBus(SL_Bus_soccer_strategy_team_communication_teamInfo* busPtr, team_communication::teamInfo const* msgPtr);


#endif
