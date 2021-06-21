#!/usr/bin/env bash
export GAME_CONTROLLER_HOME=~/catkin_ws/src/soccerbot/external/GameController
export JAVA_HOME=/usr
cd ~/catkin_ws/src/soccerbot/external/webots || exit 1

./webots ./projects/samples/contests/robocup/worlds/robocup.wbt