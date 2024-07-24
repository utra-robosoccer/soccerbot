cmake_minimum_required(VERSION 3.0.2)
project(soccer_object_localization)

add_compile_options("-std=c++17")

set(PKG_DEPS
    rospy soccer_common sensor_msgs geometry_msgs cv_bridge
)

find_package(
    catkin
    REQUIRED
    COMPONENTS ${PKG_DEPS}
)

find_package(
    OpenCV
    REQUIRED
)

catkin_package()
catkin_python_setup()
