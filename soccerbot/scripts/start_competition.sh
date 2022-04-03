#!/usr/bin/env bash

export COMPETITION=true

if [ "$ROBOCUP_ROBOT_ID" == "1" ]; then
  export X_POS=4
  export Y_POS=-3.14
  export YAW=1.57
fi

if [ "$ROBOCUP_ROBOT_ID" == "2" ]; then
  export X_POS=1
  export Y_POS=-3.14
  export YAW=1.57
fi

if [ "$ROBOCUP_ROBOT_ID" == "3" ]; then
  export X_POS=2
  export Y_POS=3.14
  export YAW=1.57
fi

if [ "$ROBOCUP_ROBOT_ID" == "4" ]; then
  export X_POS=2
  export Y_POS=3.14
  export YAW=1.57
fi

source ~/catkin_ws/devel/setup.bash
roslaunch soccerbot soccerbot.launch __ns:=robot$ROBOCUP_ROBOT_ID || sleep infinity;
