cmake_minimum_required(VERSION 2.8.3)
project(soccer_transforms)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  soccer_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/soccer_transforms_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_executable(draw_trajectories src/draw_trajectories.cpp)
add_dependencies(draw_trajectories ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(draw_trajectories ${catkin_LIBRARIES})

add_executable(vision_points_to_scan src/vision_points_to_scan.cpp)
add_dependencies(vision_points_to_scan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_points_to_scan ${catkin_LIBRARIES})
