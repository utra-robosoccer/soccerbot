cmake_minimum_required(VERSION 2.8.3)
project(team_communication)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   game_state.msg
   teamInfo.msg
   robotInfo.msg
   game_stateRes.msg
   local_model.msg
   team_local_model.msg
   team_data.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
	CATKIN_DEPENDS 
	message_runtime
	std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Game Controller Client
add_executable(gamecontroller_client src/gamecontroller_client.cpp)
add_dependencies(gamecontroller_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gamecontroller_client ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Team Communication Client
add_executable(team_communication_client src/team_communication_client.cpp)
add_dependencies(team_communication_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(team_communication_client ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## Team Communication Server
add_executable(team_communication_server src/team_communication_server.cpp)
add_dependencies(team_communication_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(team_communication_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
