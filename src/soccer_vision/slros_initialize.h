#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"
<<<<<<< HEAD
=======
#include "slros_read_image.h"
>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

<<<<<<< HEAD
=======
// For Block soccer_vision/From File/From Gazebo/Subscribe
extern SimulinkSubscriber<sensor_msgs::Image, SL_Bus_soccer_vision_sensor_msgs_Image> Sub_soccer_vision_197;

>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
void slros_node_init(int argc, char** argv);

#endif
