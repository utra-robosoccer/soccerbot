cmake_minimum_required(VERSION 3.0.2)
project(soccer_pycontrol)

find_package(catkin REQUIRED COMPONENTS
        rospy
        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)
