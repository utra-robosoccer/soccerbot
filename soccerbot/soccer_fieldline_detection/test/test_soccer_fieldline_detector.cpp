#include <gtest/gtest.h>
#include <ros/ros.h>
#include <soccer_fieldline_detection/camera.hpp>

class SoccerFieldlineDetectorFixture : public ::testing::Test {
protected:
};

TEST_F(SoccerFieldlineDetectorFixture, CameraFindFloorCoordinate) {

    Pose3 pose_msgs;
    pose_msgs.position.x = -0.5;
    pose_msgs.position.y = -0.5;
    pose_msgs.position.z = 0.5;

    pose_msgs.orientation.w = 0.8536;
    pose_msgs.orientation.x = -0.1464;
    pose_msgs.orientation.y = 0.3536;
    pose_msgs.orientation.z = 0.3536;
    pose_msgs.orientation.w = 0.8536;

    Camera cam (pose_msgs,240,360);
    Point3 p2 = cam.FindFloorCoordinate(130, 70);
    //Point3 p2 (1,2,3);
   // ASSERT();
    printf("   %f, %f, %f",p2.x,p2.y,p2.z);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_fieldline_detector");



    return RUN_ALL_TESTS();
}