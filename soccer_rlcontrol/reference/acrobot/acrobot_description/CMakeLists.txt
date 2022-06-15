cmake_minimum_required(VERSION 2.8.3)
project(acrobot_description)

find_package(
    catkin
    REQUIRED urdf xacro
)

catkin_package(CATKIN_DEPENDS urdf xacro)

xacro_add_files(
    models/acrobot.xacro OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/acrobot.urdf
    TARGET acrobot_urdf
)
