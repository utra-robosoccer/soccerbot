cmake_minimum_required(VERSION 2.8.3)
project(soccer_hardware)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        soccer_msgs
	std_msgs
	roscpp
	rospy
)

catkin_package(CATKIN_DEPENDS soccer_msgs std_msgs)

catkin_install_python(PROGRAMS src/soccer_communication/${PROJECT_NAME}.py
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

install(TARGETS draw_trajectories
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
