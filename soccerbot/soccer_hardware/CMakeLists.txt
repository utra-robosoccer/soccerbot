cmake_minimum_required(VERSION 2.8.3)
project(soccer_hardware)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        rospy
        roscpp
        std_msgs
        soccer_msgs
)

catkin_package()

catkin_install_python(PROGRAMS src/soccer_communication/soccer_hardware.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(draw_trajectories src/draw_trajectories.cpp)
add_dependencies(draw_trajectories ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(draw_trajectories ${catkin_LIBRARIES})