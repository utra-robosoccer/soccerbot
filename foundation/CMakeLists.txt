cmake_minimum_required(VERSION 2.8.3)
project(foundation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

set(PKG_DEPS
    roscpp
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES foundation
    CATKIN_DEPENDS ${PKG_DEPS}
)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Geometry Library
add_library(geometry
    src/geometry/point2.cpp
    src/geometry/point3.cpp
    src/geometry/segment2.cpp
    src/geometry/segment3.cpp
    )
add_dependencies(geometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(geometry
    ${catkin_LIBRARIES}
    )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_geometry tests/geometry/test_geometry.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
    target_link_libraries(test_geometry geometry)
endif ()