cmake_minimum_required(VERSION 3.0.2)
project(soccerbot)

find_package(
    catkin
    REQUIRED
    soccer_common
    soccer_webots
    soccer_msgs
    soccer_trajectories
    soccer_object_detection
    soccer_description
    soccer_strategy
    soccer_object_localization
    soccer_pycontrol
    soccer_hardware
)

catkin_package(CATKIN_DEPENDS soccer_msgs soccer_object_detection)
