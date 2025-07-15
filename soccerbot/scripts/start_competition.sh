#!/usr/bin/env bash

export COMPETITION=true

source ~/ros2_ws/devel/setup.bash
roslaunch soccerbot soccerbot.launch simulation:=true __ns:=robot$ROBOCUP_ROBOT_ID || sleep infinity;
