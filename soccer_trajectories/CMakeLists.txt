cmake_minimum_required(VERSION 3.0.2)
project(soccer_trajectories)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  gazebo_ros
  gazebo_ros_control
  tf2_ros
        rospy
        sensor_msgs
        rosgraph_msgs
        std_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
    head_rotator
    src/head_rotator.cpp
)
add_dependencies(head_rotator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_rotator
    ${catkin_LIBRARIES}
)
