cmake_minimum_required(VERSION 3.0.2)
project(soccer_trajectories)

add_compile_options(-std=c++11)

find_package(
    ros2
    REQUIRED
    COMPONENTS dynamic_reconfigure tf2_ros rospy sensor_msgs rosgraph_msgs
               std_msgs
)

ros2_python_setup()

ros2_package(
    CATKIN_DEPENDS
    dynamic_reconfigure
    tf2_ros
    rospy
    sensor_msgs
    rosgraph_msgs
    std_msgs
)

include_directories(${ros2_INCLUDE_DIRS})
