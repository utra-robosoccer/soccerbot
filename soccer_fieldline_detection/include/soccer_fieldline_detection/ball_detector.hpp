#pragma once

#include <ros/ros.h>

/*
 * take box of ball position in pixels
 * take transform of the camera
 * calculate location of ball in 3D
 */

class BallDetector {
    int ballPxlX;
    int ballPxlY;

    tf::StampedTransform cameraTF;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    tf2_ros::TransformBroadcaster br;

public:
    void ballDetectorCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg);
};
