cmake_minimum_required(VERSION 3.0.2)
project(soccer_embedded)

find_package(catkin REQUIRED)

catkin_package()

add_custom_target(
    soccer_embedded ALL
    COMMAND make clean 2> /dev/null && make 2> /dev/null
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Robot
)

add_custom_target(
    upload ALL
    DEPENDS soccer_embedded
    COMMAND ./scripts/flash.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
