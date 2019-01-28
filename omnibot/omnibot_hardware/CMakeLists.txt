cmake_minimum_required(VERSION 2.8.3)
project(omnibot_hardware)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  rospy
  std_msgs
  omnibot_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS bin/omnibot_hardware
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

