#pragma once

#include <soccer_fieldline_detection/camera.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

/* The soccer fieldine detector class (TODO)
 * 1. add geometry_msgs/Pose2d into the Cmakelist.txt and package.xml of soccer_foundations
 * 2. IN soccer_foundation in the geometry library, create a class called Pose3D that inherits from geometry_msgs::Pose2D
 * 3. Create Quaternion to RPY function in Pose2D, And then create getYaw, getPitch, getRoll functions in the class, use this library http://wiki.ros.org/tf2/Tutorials/Quaternions. And create a test for it
 * 4. Create the transformation matrix function that returns a 4x4 transformation matrix from the rpy and quaternion. Google the math
 * 5. Create the image class with the data members being the same from the matlab code
 * 5. Create the camera class with the data members being the same from the matlab code
 * 6. Create the draw frame test function (ask me later)
 * */


/*
 * int test create image_transport pub
 * use opncv to load video file
 * use image_transport to convert video to image_transport and publish video
 * result subscriber print received image
 */
// Camera contains an Image, Camera contains a Pose2d

class SoccerFieldlineDetector {
    ros::NodeHandle nh;
    image_transport::Subscriber image_subscriber;

public:
    SoccerFieldlineDetector();

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
};