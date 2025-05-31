cmake_minimum_required(VERSION 3.0.2)
project(soccer_object_localization)

add_compile_options("-std=c++17")

set(PKG_DEPS
    rospy soccer_common sensor_msgs geometry_msgs cv_bridge
)

find_package(
    ros2
    REQUIRED
    COMPONENTS ${PKG_DEPS}
)

find_package(
    OpenCV
    REQUIRED
)

ros2_package(CATKIN_DEPENDS ${PKG_DEPS})
ros2_python_setup()
