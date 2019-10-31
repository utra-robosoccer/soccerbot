cmake_minimum_required(VERSION 2.8.3)
project(soccer_description)

find_package(catkin REQUIRED urdf xacro)

catkin_package(
    CATKIN_DEPENDS urdf xacro
)

xacro_add_files(models/soccerbot.xacro OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/models/soccerbot.urdf TARGET soccer_urdf)
xacro_add_files(models/soccerbot_stl.xacro OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/models/soccerbot_stl.urdf TARGET media_files)
