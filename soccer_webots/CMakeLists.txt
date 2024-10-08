cmake_minimum_required(VERSION 3.0.2)
project(soccer_webots)

add_compile_options(-std=c++11)
# include(FindProtobuf)
find_package(
    Protobuf
    REQUIRED
)

find_package(
    catkin
    REQUIRED
    COMPONENTS dynamic_reconfigure
               tf2_ros
               roscpp
               rospy
               sensor_msgs
               rosgraph_msgs
               std_msgs
)

catkin_python_setup()
catkin_package(
    CATKIN_DEPENDS
    dynamic_reconfigure
    tf2_ros
    roscpp
    rospy
    sensor_msgs
    rosgraph_msgs
    std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

protobuf_generate_python(PROTO_PY src/soccer_webots/messages.proto)
add_custom_target(robocup_api_proto ALL DEPENDS ${PROTO_PY})
add_custom_command(
    TARGET robocup_api_proto
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_PY}
            ${CMAKE_SOURCE_DIR}/src/soccer_webots
)
