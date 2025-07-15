#!/usr/bin/env bash

rm -rf *.bag
rm -rf *.active
scp robot1@192.168.0.101:/home/robot1/ros2_ws/src/soccerbot/soccerbot/bags/*.bag .
scp robot1@192.168.0.101:/home/robot1/ros2_ws/src/soccerbot/soccerbot/bags/*.active .
