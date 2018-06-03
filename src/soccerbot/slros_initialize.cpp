#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "soccerbot";

// For Block soccerbot/Subscribe
SimulinkSubscriber<sensor_msgs::CompressedImage, SL_Bus_soccerbot_sensor_msgs_CompressedImage> Sub_soccerbot_97;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

