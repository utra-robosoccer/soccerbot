#include <iostream>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <cmath>
#include <soccer_route_planner/route_planner_node.hpp>
#include <geometry_msgs/Pose2D.h>

class RoutePlannerTestBase : public ::testing::Test {
protected:
    virtual void SetUp() {

    }

    virtual void TearDown() {}

protected:
    ros::NodeHandle nh;
};

TEST_F(RoutePlannerTestBase, route_planner_test) {
    ros::spin();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "route_planner_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

