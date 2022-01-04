cmake_minimum_required(VERSION 3.0.2)
project(soccer_strategy)

find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        soccer_msgs
        soccer_pycontrol
        soccer_geometry
        rospy
        tf
        )

catkin_package()

protobuf_generate_python(PROTO_PY src/game_controller/RobocupProtocol/robocup_extension.proto)
add_custom_target(soccer_strategy ALL DEPENDS ${PROTO_PY})
add_custom_command(
  TARGET soccer_strategy POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_PY} ${CMAKE_SOURCE_DIR}/src/game_controller
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
catkin_python_setup()