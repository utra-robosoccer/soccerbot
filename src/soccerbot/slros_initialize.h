#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"
#include "slros_read_image.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block soccerbot/Subscribe
extern SimulinkSubscriber<sensor_msgs::CompressedImage, SL_Bus_soccerbot_sensor_msgs_CompressedImage> Sub_soccerbot_97;

void slros_node_init(int argc, char** argv);

#endif
