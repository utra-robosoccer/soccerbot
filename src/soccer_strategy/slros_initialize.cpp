#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "soccer_strategy";

// For Block soccer_strategy/Subscribe1
SimulinkSubscriber<sensor_msgs::Imu, SL_Bus_soccer_strategy_sensor_msgs_Imu> Sub_soccer_strategy_1090;

// For Block soccer_strategy/Subscribe2
SimulinkSubscriber<team_communication::game_state, SL_Bus_soccer_strategy_team_communication_game_state> Sub_soccer_strategy_1306;

// For Block soccer_strategy/Publish
SimulinkPublisher<soccer_msgs::RobotGoal, SL_Bus_soccer_strategy_soccer_msgs_RobotGoal> Pub_soccer_strategy_103;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

