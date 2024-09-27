#!/usr/bin/env bash
export GAME_CONTROLLER_HOME=~/catkin_ws/src/soccerbot/external/GameController/
export JAVA_HOME=/usr
export WEBOTS_HOME=/usr/local/webots

cd ~/catkin_ws/src/soccerbot/external || exit 1
webots ./hlvs_webots/worlds/robocup.wbt
