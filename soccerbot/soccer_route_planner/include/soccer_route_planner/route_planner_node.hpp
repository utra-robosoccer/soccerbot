#pragma once

#include <ros/ros.h>
#include <soccer_msgs/Waypoints.h>
#include <soccer_msgs/MapOverview.h>

class RoutePlannerNode {
public:
    RoutePlannerNode();
    // Map map;
private:
    ros::Subscriber robotCommandSubscriber;
    ros::Subscriber mapOverviewSubscriber;
    ros::Publisher<soccer_msgs::Waypoints> waypointPublisher;

    // upon overview subscription update Map
    void mapOverviewCallback(/* Map overview message */);

    // Upon subscription, first use the map to find a path, and then publish using the waypoint publisher
    void robotCommandCallback(/* Robot Command message */);
};