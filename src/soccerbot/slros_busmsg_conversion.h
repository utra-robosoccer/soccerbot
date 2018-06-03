#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <ros/time.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>
#include "soccerbot_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(geometry_msgs::Point* msgPtr, SL_Bus_soccerbot_geometry_msgs_Point const* busPtr);
void convertToBus(SL_Bus_soccerbot_geometry_msgs_Point* busPtr, geometry_msgs::Point const* msgPtr);

void convertFromBus(ros::Time* msgPtr, SL_Bus_soccerbot_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_soccerbot_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(sensor_msgs::CompressedImage* msgPtr, SL_Bus_soccerbot_sensor_msgs_CompressedImage const* busPtr);
void convertToBus(SL_Bus_soccerbot_sensor_msgs_CompressedImage* busPtr, sensor_msgs::CompressedImage const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_soccerbot_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_soccerbot_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);


#endif
