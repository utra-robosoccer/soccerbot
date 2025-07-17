cmake_minimum_required(VERSION 3.0.2)
project(soccer_common)

add_compile_options(-std=c++17)

set(PKG_DEPS
    rospy geometry_msgs soccer_msgs
)

find_package(
    ros2
    REQUIRED ${PKG_DEPS}
)

ros2_package(CATKIN_DEPENDS soccer_msgs ${PKG_DEPS})

# ##############################################################################
# Build ##
# ##############################################################################
include_directories(${ros2_INCLUDE_DIRS})

ros2_python_setup()
