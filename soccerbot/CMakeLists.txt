cmake_minimum_required(VERSION 2.8.3)
project(soccerbot)

find_package(catkin REQUIRED COMPONENTS 
	amcl 
	darknet_ros 
	map_server 
	soccer_common 
	soccer_description 
	soccer_hardware 
	usb_cam
)

catkin_package()

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS scripts/startup.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

