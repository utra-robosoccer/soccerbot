#pragma once

#include <soccer_fieldline_detection/camera.hpp>

/* The soccer fieldine detector class (TODO)
 * 1. add geometry_msgs/Pose2d into the Cmakelist.txt and package.xml of soccer_foundations
 * 2. IN soccer_foundation in the geometry library, create a class called Pose3D that inherits from geometry_msgs::Pose2D
 * 3. Create Quaternion to RPY function in Pose2D, And then create getYaw, getPitch, getRoll functions in the class, use this library http://wiki.ros.org/tf2/Tutorials/Quaternions. And create a test for it
 * 4. Create the transformation matrix function that returns a 4x4 transformation matrix from the rpy and quaternion. Google the math
 * 5. Create the image class with the data members being the same from the matlab code
 * 5. Create the camera class with the data members being the same from the matlab code
 * 6. Create the draw frame test function (ask me later)
 * */

// Camera contains an Image, Camera contains a Pose2d