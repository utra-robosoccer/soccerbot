cmake_minimum_required(VERSION 3.0.2)
project(soccer_firmware_interface)

find_package(
    catkin
    REQUIRED
    COMPONENTS rospy std_msgs sensor_msgs geometry_msgs control_msgs
)

catkin_package()
catkin_python_setup()
