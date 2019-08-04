#include <iostream>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <soccer_route_planner/route_planner_node.hpp>

class RoutePlannerTest : public ::testing::Test {
protected:
    virtual void SetUp() {

    }

    virtual void TearDown() {}

    ros::NodeHandle _nh;
};

TEST_F(RoutePlannerBase, route_planner_test) {
    // Create fake map overview and robot command publisher, and then publish a fake message

    // Subscribe to ros::Publisher<soccer_msgs::Waypoints> waypointPublisher;
    // And then add some tests to see if the recieved value is correct
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "route_planner_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
