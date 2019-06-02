cmake_minimum_required(VERSION 2.8.3)
project(soccer_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

add_message_files(
    FILES
    RobotGoal.msg
    RobotState.msg
    RobotCommand.msg
    RobotEstimate.msg
    MapOverview.msg
    MapOverviewIndividual.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS message_generation roscpp rospy std_msgs message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)