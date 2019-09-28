#include <gtest/gtest.h>
#include <ros/ros.h>
#include <soccer_fieldline_detection/camera.hpp>

class SoccerFieldlineDetectorFixture : public ::testing::Test {
protected:
};

TEST_F(SoccerFieldlineDetectorFixture, CameraFindFloorCoordinate) {
    Camera camera();
    Point2 p1  (3,4);
    Point3 p2 = Camera::FindFloorCoordinate(3,4);
    ASSERT(3dpoint = 3dpoint);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_fieldline_detector");
    return RUN_ALL_TESTS();
}