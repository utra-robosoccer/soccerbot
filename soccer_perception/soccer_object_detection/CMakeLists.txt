cmake_minimum_required(VERSION 3.0.2)
project(soccer_object_detection)

find_package(
    catkin
    REQUIRED
    COMPONENTS rospy
)

catkin_package(CATKIN_DEPENDS)
catkin_python_setup()

include_directories(${catkin_INCLUDE_DIRS})
