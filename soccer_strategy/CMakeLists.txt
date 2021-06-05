cmake_minimum_required(VERSION 3.0.2)
project(soccer_strategy)

find_package(catkin REQUIRED COMPONENTS
        soccer_msgs
        rospy
        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)
