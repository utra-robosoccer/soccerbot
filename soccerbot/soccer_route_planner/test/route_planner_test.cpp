#include <iostream>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <cmath>
#include <soccer_route_planner/route_planner_node.hpp>
#include <soccer_msgs/MapOverview.h>
#include <soccer_msgs/RobotCommand.h>

class RoutePlannerTestBase : public ::testing::Test {
protected:
    virtual void SetUp() {

    }

    virtual void TearDown() {}

public:
    void waypointsCallback(const soccer_msgs::WaypointsConstPtr &waypoint) {
//        ASSERT_TRUE(waypoint->poseActions[0].pose);
//        ASSERT_TRUE(waypoint->poseActions[0].actionLabel == 1);
//        ASSERT_TRUE(waypoint->poseActions[0].duration);
    }
protected:
    ros::NodeHandle nh;
    ros::Publisher fakeMapOverviewPublisher;
    ros::Publisher fakeRobotCommandPublisher;
    ros::Subscriber fakeWaypointSubscriber;
};

TEST_F(RoutePlannerTestBase, route_planner_test) {
    // Create fake map overview and robot command publisher, and then publish a fake message
    fakeMapOverviewPublisher = nh.advertise<soccer_msgs::MapOverview>("map_overview", 1);

    // Building fake mapOverview message.
    soccer_msgs::MapOverview mapOverview;
    mapOverview.header.stamp = ros::Time::now();

    mapOverview.ballPose.x = 1.f;
    mapOverview.ballPose.y = 1.f;
    mapOverview.ballPose.theta = M_PI_2;

    soccer_msgs::RobotEstimate robotEstimate;
    robotEstimate.isAlly = true;
    robotEstimate.isGoalie = false;
    robotEstimate.isStanding = true;
    robotEstimate.robotID = 0;

    robotEstimate.pose.x = 0.f;
    robotEstimate.pose.y = 0.f;
    robotEstimate.pose.theta = M_PI_4;

    mapOverview.estimates.push_back(robotEstimate);

    while(fakeMapOverviewPublisher.getNumSubscribers() == 0) {
        ros::Duration(1.0f).sleep();
    }
    fakeMapOverviewPublisher.publish(mapOverview);


    // In a similar fashion, publish a robot command, to the route planner, and then spin.
    // Subscribe to wayPointPublisher, and then use ASSERT_TRUE statements to verify the output
    fakeRobotCommandPublisher = nh.advertise<soccer_msgs::RobotCommand>("robot_command", 1);

    soccer_msgs::RobotCommand robotCommand;
    robotCommand.header.stamp = ros::Time::now();

    robotCommand.command = 0;

    robotCommand.currPose.theta = M_PI_4;
    robotCommand.currPose.x = 0.f;
    robotCommand.currPose.y = 0.f;

    robotCommand.destBall.y = 1.f;
    robotCommand.destBall.x = 1.f;
    robotCommand.destBall.theta = M_PI_2;

    robotCommand.destPose.theta = M_PI_2;
    robotCommand.destPose.x = 1.f;
    robotCommand.destPose.y = 1.f;

    robotCommand.leg = 0;

    while(fakeRobotCommandPublisher.getNumSubscribers() == 0) {
        ros::Duration(1.0f).sleep();
    }
    fakeRobotCommandPublisher.publish(robotCommand);

    fakeWaypointSubscriber = nh.subscribe("waypoints", 1, &RoutePlannerTestBase::waypointsCallback, static_cast<RoutePlannerTestBase*>(this));

    ros::spin();

    // Subscribe to ros::Publisher<soccer_msgs::Waypoints> waypointPublisher;
    // And then add some tests to see if the recieved value is correct
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "route_planner_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

