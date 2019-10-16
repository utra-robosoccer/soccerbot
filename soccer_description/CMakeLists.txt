cmake_minimum_required(VERSION 2.8.3)
project(soccer_description)

find_package(catkin REQUIRED COMPONENTS 
	dynamic_reconfigure
	gazebo_ros
	gazebo_ros_control
	urdf
	xacro
)

catkin_package(
    CATKIN_DEPENDS dynamic_reconfigure urdf xacro 
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS scripts/kill_gazebo.sh
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

xacro_add_files(models/soccerbot.xacro OUTPUT ${CATKIN_PACKAGE_BIN_DESTINATION}/models/soccerbot.urdf TARGET soccer_urdf)
xacro_add_files(models/soccerbot_stl.xacro OUTPUT i${CATKIN_PACKAGE_BIN_DESTINATION}/models/soccerbot_stl.urdf TARGET media_files)
