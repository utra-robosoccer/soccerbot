cmake_minimum_required(VERSION 3.0.2)
project(soccer_object_detection)

find_package(
    ros2
    REQUIRED
    COMPONENTS rospy
)

ros2_package(CATKIN_DEPENDS)
ros2_python_setup()

include_directories(${ros2_INCLUDE_DIRS})
