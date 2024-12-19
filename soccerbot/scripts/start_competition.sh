#!/usr/bin/env bash

export COMPETITION=false

source ~/catkin_ws/devel/setup.bash
roslaunch soccerbot soccerbot.launch simulation:=true __ns:=robot$ROBOCUP_ROBOT_ID --wait || sleep infinity;
