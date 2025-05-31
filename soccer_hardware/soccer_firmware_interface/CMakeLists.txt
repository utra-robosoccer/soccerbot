cmake_minimum_required(VERSION 3.0.2)
project(soccer_firmware_interface)

find_package(
    ros2
    REQUIRED
    COMPONENTS rospy std_msgs sensor_msgs geometry_msgs control_msgs
)
ros2_package(
    CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    control_msgs
)
ros2_python_setup()
