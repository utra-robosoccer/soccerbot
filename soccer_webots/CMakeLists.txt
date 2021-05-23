cmake_minimum_required(VERSION 3.0.2)
project(soccer_webots)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        tf2_ros
        roscpp
        rospy
        sensor_msgs
        rosgraph_msgs
        std_msgs

)

catkin_python_setup()
catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

