cmake_minimum_required(VERSION 2.8.3)
project(soccer_control_center)

set(PKG_DEPS

)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

catkin_python_setup()

find_library(cpprestsdk REQUIRED)
find_package(jsoncpp REQUIRED)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS ${PKG_DEPS}
    LIBRARIES ${PROJECT_NAME}
)

#############
##  Build  ##
#############
