#pragma once

#include <geometry/pose3.hpp>
#include <geometry/point3.hpp>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

class Camera {

    Pose3 pose;

    // Values for the C920 camera
    float resolution_x = 0;
    float resolution_y = 0;

private:
    float diagonal_fov = 1.57; // radians (field of view)
    float focal_length = 3.67; // mm

    //For creating the points
    float max_line_distance = 10;   // Maximum distance to project the current line, if the endpoint is above the horizon

    ros::Subscriber cameraInfoSubscriber;
    ros::NodeHandle nh;

public:
    bool ready() const {
        return resolution_x != 0 && resolution_y != 0;
    }

    explicit Camera(const Pose3 &pose);

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info);

    void setPose();

    void DrawPixelRayTrace(int pixel_y, int pixel_x);

    Point3 FindFloorCoordinate(int pos_x, int pos_y);

    float VerticalFOV();

    float HorizontalFOV();

    float ImageSensorHeight();

    float ImageSensorWidth();

    float PixelHeight();

    float PixelWidth();

    float ImageSensorLocation_X(int pos_x, int pos_y);

    float ImageSensorLocation_Y(int pos_x, int pos_y);
};
