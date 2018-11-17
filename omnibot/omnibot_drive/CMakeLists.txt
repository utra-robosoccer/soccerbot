cmake_minimum_required(VERSION 2.8.3)
project(omnibot_drive)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS roscpp geometry_msgs std_msgs
        DEPENDS
)

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
        scripts/twist_to_motors.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark all other useful stuff for installation
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
