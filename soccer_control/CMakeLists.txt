cmake_minimum_required(VERSION 2.8.3)
project(soccer_control)

find_package(catkin REQUIRED)

catkin_package()

add_custom_target(
    soccer_control ALL
    COMMAND ./scripts/build.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)