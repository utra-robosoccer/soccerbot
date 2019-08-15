#pragma once

#include <ros/ros.h>
#include <soccer_msgs/Waypoints.h>
#include <soccer_msgs/MapOverview.h>
#include <soccer_msgs/RobotCommand.h>

class RoutePlannerNode {
public:
    RoutePlannerNode();
    // Map map;
private:
    ros::NodeHandle nh;
    ros::Subscriber robotCommandSubscriber;
    ros::Subscriber mapOverviewSubscriber;
    ros::Publisher waypointPublisher;

    soccer_msgs::MapOverviewPtr mapOverviewStore;
    soccer_msgs::RobotCommandPtr robotCommandStore;

    // upon overview subscription update Map
    void mapOverviewCallback(const soccer_msgs::MapOverviewConstPtr& mapOverview);
    void robotCommandCallback(const soccer_msgs::RobotCommandConstPtr& robotCommand);

    soccer_msgs::Waypoints createWaypoints();

    void publishWaypoints();

    // Upon subscription, first use the map to find a path, and then publish using the waypoint publisher
//    void robotCommandCallback(/* Robot Command message */);
};