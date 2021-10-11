cmake_minimum_required(VERSION 3.0.2)
project(soccer_geometry)

add_compile_options(-std=c++17)

set(PKG_DEPS
    rospy
    geometry_msgs
    )

find_package(catkin REQUIRED ${PKG_DEPS})

catkin_package()

catkin_python_setup()
