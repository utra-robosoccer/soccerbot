#!/bin/bash

############################################################################
# This script should be used to start the robocup stack for the simulator. #
# In the docker container, it is copied to /usr/local/bin/start. Therefore #
# it can be started using, for example, "start goalie 0" as the dockerCmd. #
############################################################################
#exec source ~/catkin_ws/devel/setup.sh
#exec catkin build soccerbot
exec roslaunch  soccerbot soccerbot_multi.launch competition:=true single:=true fake_localization:=false robot_name:=$1 x_pos:=$2 y_pos:=$3 a_pos:=$4