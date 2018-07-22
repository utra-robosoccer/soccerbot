#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "soccer_vision";

<<<<<<< HEAD
=======
// For Block soccer_vision/From File/From Gazebo/Subscribe
SimulinkSubscriber<sensor_msgs::Image, SL_Bus_soccer_vision_sensor_msgs_Image> Sub_soccer_vision_197;

>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

