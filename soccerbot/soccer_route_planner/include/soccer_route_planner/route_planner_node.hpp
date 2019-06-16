#pragma once

#include <ros/ros.h>
#include <soccer_msgs/Waypoints.h>
#include <soccer_msgs/MapOverview.h>

class RoutePlannerNode {
public:
    RoutePlannerNode();

private:
    ros::Subscriber robotCommandSubscriber;
    ros::Publisher<soccer_msgs::Waypoints> waypointPublisher;
};