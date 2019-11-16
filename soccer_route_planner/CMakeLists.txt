cmake_minimum_required(VERSION 2.8.3)
project(soccer_route_planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(SOURCE_FILES src/Graph.cpp)

find_package(catkin REQUIRED
    roscpp
    soccer_msgs
)

catkin_package()
include_directories(
        include
  ${catkin_INCLUDE_DIRS}
        ../../../devel/include # Hack
)

add_executable(route_planner src/route_planner_node.cpp src/Graph.cpp src/RRTStar.cpp src/Map.cpp)
add_dependencies(route_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(route_planner
    ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin build soccer_route_planner --verbose --no-deps --catkin-make-args run_tests
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    include_directories(${catkin_INCLUDE_DIRS})

    add_rostest_gtest(route_planner_test test/route_planner_test.test test/route_planner_test.cpp)
    target_link_libraries(route_planner_test ${catkin_LIBRARIES})

endif()