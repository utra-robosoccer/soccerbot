cmake_minimum_required(VERSION 3.0.2)
project(soccer_pycontrol)

find_package(
    ros2
    REQUIRED
    COMPONENTS rospy soccer_common
)

ros2_package()
ros2_python_setup()

include_directories(${ros2_INCLUDE_DIRS})
