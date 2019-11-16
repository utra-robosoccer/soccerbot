#pragma once

#include <soccer_route_planner/Graph.hpp>
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
    void mapOverviewCallback(const soccer_msgs::MapOverviewPtr &mapOverview);
    void robotCommandCallback(const soccer_msgs::RobotCommandPtr& robotCommand);

    // Waypoints functions
    soccer_msgs::Waypoints createWaypoints();

    void publishWaypoints(soccer_msgs::Waypoints);

    // RRT* algorithm / primitives below.
    Graph RRTstar(int r, int n);
    void sample();
    void nearestNeighbour();
    void nearVertices();
    void steering();
    void collisionTest();

    // Upon subscription, first use the map to find a path, and then publish using the waypoint publisher
//    void robotCommandCallback(/* Robot Command message */);
};