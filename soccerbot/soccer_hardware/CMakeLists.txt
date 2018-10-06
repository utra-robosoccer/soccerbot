cmake_minimum_required(VERSION 2.8.3)
project(soccer_hardware)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  rospy
  roscpp
  std_msgs
  message_generation
  soccer_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(soccer_hardware_test src/soccer_hardware_test.cpp)
add_dependencies(soccer_hardware_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(soccer_hardware_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_trajectories src/test_trajectories.cpp)
add_dependencies(test_trajectories ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_trajectories ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(draw_trajectories src/draw_trajectories.cpp)
add_dependencies(draw_trajectories ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(draw_trajectories ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

catkin_install_python(PROGRAMS nodes/soccer_hardware.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
