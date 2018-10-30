cmake_minimum_required(VERSION 2.8.3)
project(omnibot_control)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
)

catkin_package()
