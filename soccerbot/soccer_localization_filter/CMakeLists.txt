cmake_minimum_required(VERSION 2.8.3)
project(soccer_localization_filter)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        soccer_msgs
        roscpp
)

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(localization_filter src/soccer_localization_filter_node.cpp)

add_dependencies(localization_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(localization_filter
        ${catkin_LIBRARIES}
)
