#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"
#include "slros_read_image.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block soccer_vision/From File/From Gazebo/Subscribe
extern SimulinkSubscriber<sensor_msgs::Image, SL_Bus_soccer_vision_sensor_msgs_Image> Sub_soccer_vision_197;

void slros_node_init(int argc, char** argv);

#endif
