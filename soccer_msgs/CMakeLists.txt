cmake_minimum_required(VERSION 2.8.3)
project(soccer_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
  geometry_msgs
  nav_msgs
)

add_message_files(
    FILES
    EndAction.msg
    RobotCommand.msg
    RobotPath.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
)

catkin_package(
 CATKIN_DEPENDS message_generation std_msgs message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)