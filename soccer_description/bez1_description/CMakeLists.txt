cmake_minimum_required(VERSION 3.0.2)
project(bez1_description)

find_package(
    catkin
    REQUIRED urdf xacro
)

catkin_package(CATKIN_DEPENDS urdf xacro)

xacro_add_files(
    urdf/bez1.xacro OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/urdf/bez1.urdf TARGET
    media_files
)
