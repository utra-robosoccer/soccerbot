cmake_minimum_required(VERSION 2.8.3)
project(soccer_monitor)

##################
## Dependencies ##
##################
find_package(catkin REQUIRED)

catkin_package()

add_custom_target( soccer_monitor_install ALL
  COMMAND ./scripts/install.sh
  WORKING_DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}
)