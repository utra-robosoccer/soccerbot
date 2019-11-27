cmake_minimum_required(VERSION 2.8.3)
project(soccer_embedded)

find_package(catkin REQUIRED)

catkin_package()

add_custom_target(
    soccer_embedded ALL
    COMMAND make clean && make
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Robot
)

add_custom_target(
    upload ALL
    DEPENDS soccer_embedded
    COMMAND openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg -c "program ./Build/F4/Robot_F4.elf verify reset exit"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Robot
)
