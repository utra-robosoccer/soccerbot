cmake_minimum_required(VERSION 2.8.3)
project(soccer_matlab)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
    scripts/test_computer_vision.sh
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)