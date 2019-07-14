#pragma once

#include <geometry/pose3.hpp>
#include <cmath>

class Camera {

    Pose3 pose;

    // Values for the C920 camera
    int resolution_y = 240;
    int resolution_x = 360;
    float diagonal_fov = 1.36; // radians (field of view)
    float focal_length = 3.67; // mm

    //For creating the points
    float max_line_distance = 10;   // Maximum distance to project the current line, if the endpoint is above the horizon

    // Laser scan data laser_scan_angle_delta = deg2rad(3);
    double angmin = 0;
    double angmax = M_PI;

    // Camera Image (type Camera Image)
    // image

public:
    Camera(const Pose3 &pose, int resolution_y, int resolution_x);

    void DrawPixelRayTrace(int pixel_y, int pixel_x);
};
