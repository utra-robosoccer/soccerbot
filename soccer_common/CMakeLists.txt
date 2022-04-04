cmake_minimum_required(VERSION 3.0.2)
project(soccer_common)

add_compile_options(-std=c++17)

set(PKG_DEPS
    rospy geometry_msgs soccer_msgs
)

find_package(
    catkin
    REQUIRED ${PKG_DEPS}
)

catkin_package(CATKIN_DEPENDS soccer_msgs)

# ##############################################################################
# Build ##
# ##############################################################################
include_directories(${catkin_INCLUDE_DIRS})

catkin_python_setup()
