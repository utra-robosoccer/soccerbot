#!/usr/bin/env bash

export COMPETITION=true

if [ "$ROBOCUP_ROBOT_ID" == "1" ]; then
  export X_POS=-4
  export Y_POS=-3.15
  export YAW=1.57
fi

if [ "$ROBOCUP_ROBOT_ID" == "2" ]; then
  export X_POS=-1
  export Y_POS=-3.15
  export YAW=1.57
fi

if [ "$ROBOCUP_ROBOT_ID" == "3" ]; then
  export X_POS=-1
  export Y_POS=3.15
  export YAW=-1.57
fi

if [ "$ROBOCUP_ROBOT_ID" == "4" ]; then
  export X_POS=-4
  export Y_POS=3.15
  export YAW=-1.57
fi

source ~/catkin_ws/devel/setup.bash
roslaunch soccerbot soccerbot.launch simulation:=true __ns:=robot$ROBOCUP_ROBOT_ID || sleep infinity;
