#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block soccer_strategy/Subscribe1
extern SimulinkSubscriber<sensor_msgs::Imu, SL_Bus_soccer_strategy_sensor_msgs_Imu> Sub_soccer_strategy_1090;

// For Block soccer_strategy/Subscribe2
extern SimulinkSubscriber<team_communication::game_state, SL_Bus_soccer_strategy_team_communication_game_state> Sub_soccer_strategy_1306;

// For Block soccer_strategy/Publish
extern SimulinkPublisher<soccer_msgs::RobotGoal, SL_Bus_soccer_strategy_soccer_msgs_RobotGoal> Pub_soccer_strategy_103;

void slros_node_init(int argc, char** argv);

#endif
