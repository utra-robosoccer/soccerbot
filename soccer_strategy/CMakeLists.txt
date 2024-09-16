cmake_minimum_required(VERSION 3.0.2)
project(soccer_strategy)

find_package(
    Protobuf
    REQUIRED
)

find_package(
    catkin
    REQUIRED
    COMPONENTS soccer_msgs soccer_pycontrol soccer_common rospy tf
)

catkin_package(CATKIN_DEPENDS soccer_msgs tf)

# protobuf_generate_python( PROTO_PY
# src/soccer_strategy/communication/robot_state.proto )
# add_custom_target(soccer_strategy ALL DEPENDS ${PROTO_PY}) add_custom_command(
# TARGET soccer_strategy POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_PY}
# ${CMAKE_SOURCE_DIR}/src/soccer_strategy/communication )

include_directories(${catkin_INCLUDE_DIRS})
catkin_python_setup()
