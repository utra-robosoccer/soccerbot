#pragma once

#include <soccer_route_planner/graph.hpp>
#include <ros/ros.h>
#include <soccer_msgs/RobotCommand.h>
#include <unordered_map>

class RoutePlannerNode {
public:
    RoutePlannerNode();

private:
    ros::NodeHandle nh;
    ros::Subscriber robotCommandSubscriber;
    ros::Subscriber robotPoseSubscriber;
    ros::Publisher waypointPublisher;
    ros::Publisher graphPublisher;

    std::unordered_map<int, geometry_msgs::Pose2DPtr> robot_poses;

    // Subscriber to robot information to build robot information
    void robotPoseCallback(const geometry_msgs::Pose2DPtr& robotPose);

    // upon overview subscription update Map
    void robotCommandCallback(const soccer_msgs::RobotCommandPtr& robotCommand);
};