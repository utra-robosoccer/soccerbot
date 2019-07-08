cmake_minimum_required(VERSION 2.8.3)
project(soccer_fieldline_detection)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    foundation
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${PKG_DEPS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}

)

add_executable(soccer_fieldline_detector src/soccer_fieldline_detector.cpp)
add_dependencies(soccer_fieldline_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(soccer_fieldline_detector
    ${catkin_LIBRARIES}
)