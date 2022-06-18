cmake_minimum_required(VERSION 3.0.2)
project(soccer_description)

find_package(
    catkin
    REQUIRED urdf xacro
)

catkin_package(CATKIN_DEPENDS urdf xacro)

xacro_add_files(
    models/bez1/soccerbot.xacro OUTPUT
    ${CMAKE_CURRENT_SOURCE_DIR}/models/bez1/soccerbot.urdf TARGET media_files
)
