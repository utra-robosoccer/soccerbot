cmake_minimum_required(VERSION 3.0.2)
project(soccer_geometry)

add_compile_options(-std=c++17)

set(PKG_DEPS
    roscpp
    geometry_msgs
    )

find_package(catkin REQUIRED ${PKG_DEPS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
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
add_library(${PROJECT_NAME}
    src/soccer_geometry/point2.cpp
    src/soccer_geometry/point3.cpp
    src/soccer_geometry/segment2.cpp
    src/soccer_geometry/segment3.cpp
    src/soccer_geometry/pose3.cpp
    src/soccer_geometry/line2.cpp
    )
add_dependencies(soccer_geometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(soccer_geometry
    ${catkin_LIBRARIES}
    )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_geometry tests/soccer_geometry/test_geometry.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tests)
    target_link_libraries(test_geometry soccer_geometry)
endif ()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE
)
