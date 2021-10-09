cmake_minimum_required(VERSION 3.0.2)
project(soccer_geometry)

add_compile_options(-std=c++17)

set(PKG_DEPS
    rospy
    geometry_msgs
    )

find_package(catkin REQUIRED ${PKG_DEPS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${PKG_DEPS}
)

catkin_python_setup()
