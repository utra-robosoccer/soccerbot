cmake_minimum_required(VERSION 3.0.2)
project(soccerbot)

find_package(catkin REQUIRED)

# Build Game Controller
execute_process(COMMAND /usr/bin/ant -v WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../external/GameController/)

catkin_package()
