cmake_minimum_required(VERSION 3.0.2)
project(soccer_localization)

find_package(
    catkin
    REQUIRED
    COMPONENTS rospy soccer_common
)

catkin_package()
catkin_python_setup()

include_directories(${catkin_INCLUDE_DIRS})
