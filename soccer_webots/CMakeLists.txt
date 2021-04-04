cmake_minimum_required(VERSION 3.0.2)
project(soccer_webots)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        dynamic_reconfigure
        gazebo_ros
        gazebo_ros_control
        tf2_ros
        roscpp
        rospy

)

catkin_python_setup()
catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(
        controller_name
        src/controller_name.cpp
)
add_dependencies(controller_name ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller_name
        ${catkin_LIBRARIES}
        )
add_executable(
        motor_mapping
        src/motor_mapping.cpp
)
add_dependencies(motor_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_mapping
        ${catkin_LIBRARIES}
        )
