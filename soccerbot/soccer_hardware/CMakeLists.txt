cmake_minimum_required(VERSION 2.8.3)
project(soccer_hardware)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  soccer_msgs
)

catkin_package()

catkin_install_python(PROGRAMS src/soccer_communication/soccer_hardware.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

