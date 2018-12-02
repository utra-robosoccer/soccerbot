cmake_minimum_required(VERSION 2.8.3)
project(soccer_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
