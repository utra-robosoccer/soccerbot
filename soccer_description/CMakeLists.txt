cmake_minimum_required(VERSION 2.8.3)
project(soccer_description)

find_package(catkin REQUIRED urdf xacro)

catkin_package(
    CATKIN_DEPENDS urdf xacro
)

install(DIRECTORY models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY meshes
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})