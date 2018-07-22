#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "soccer_vision";

// For Block soccer_vision/From File/From Gazebo/Subscribe
SimulinkSubscriber<sensor_msgs::Image, SL_Bus_soccer_vision_sensor_msgs_Image> Sub_soccer_vision_197;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

