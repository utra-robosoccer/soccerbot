#!/bin/bash

############################################################################
# This script should be used to start the robocup stack for the simulator. #
# In the docker container, it is copied to /usr/local/bin/start. Therefore #
# it can be started using, for example, "start goalie 0" as the dockerCmd. #
############################################################################
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
#exec catkin build soccerbot
##########
# sudo docker run -e ROBOCUP_SIMULATOR_ADDR=127.0.0.1:10021 -e ROBOCUP_TEAM_COLOR=blue -e ROBOCUP_ROBOT_ID=1 -e ROBOCUP_GAMECONTROLLER_IP=127.0.1.1 -e ROBOCUP_MIRROR_SERVER_IP=127.0.0.1 --net="host" utrarobosoccer/soccerbot:latest bash ./src/soccerbot/soccerbot/scripts/start_competition.sh robot1 -4 -3.6 1.57
# sudo docker run -e ROBOCUP_SIMULATOR_ADDR=127.0.0.1:10021 -e ROBOCUP_TEAM_COLOR=blue -e ROBOCUP_ROBOT_ID=1 -e ROBOCUP_GAMECONTROLLER_IP=127.0.1.1 --net="host" utrarobosoccer/soccerbot:latest bash ./start_competition.sh robot1
##########


ROBOT_NAME=$1
COMPETITION=true
TEAM_ID=16
ENABLE_PYBULLET=false
if [[ "blue" = "$ROBOCUP_TEAM_COLOR" ]]
then
  if [[ "robot1" = "$1" ]]
  then
      GOALIE=true
      X_POS=-4
      Y_POS=-3.06
      ANGLE=1.57
  else
      GOALIE=false
  fi

  if [[ "robot2" = "$1" ]]
  then
      X_POS=-1
      Y_POS=-3.06
      ANGLE=1.57

  fi

  if [[ "robot3" = "$1" ]]
  then
      X_POS=-4
      Y_POS=3.06
      ANGLE=-1.57

  fi

  if [[ "robot4" = "$1" ]]
  then
      X_POS=-1
      Y_POS=3.06
      ANGLE=-1.57

  fi

fi

if [[ "red" = "$ROBOCUP_TEAM_COLOR" ]]
then
  if [[ "robot1" = "$1" ]]
  then
      GOALIE=true
      X_POS=4
      Y_POS=-3.06
      ANGLE=1.57
  else
      GOALIE=false
  fi

  if [[ "robot2" = "$1" ]]
  then
      X_POS=1
      Y_POS=-3.06
      ANGLE=1.57

  fi

  if [[ "robot3" = "$1" ]]
  then
      X_POS=4
      Y_POS=3.06
      ANGLE=-1.57

  fi

  if [[ "robot4" = "$1" ]]
  then
      X_POS=1
      Y_POS=3.06
      ANGLE=-1.57

  fi

fi
echo "ROBOCUP_ROBOT_ID: $ROBOCUP_ROBOT_ID"
echo "ROBOCUP_TEAM_COLOR: $ROBOCUP_TEAM_COLOR"
echo "ROBOCUP_SIMULATOR_ADDR: $ROBOCUP_SIMULATOR_ADDR"
echo "ROBOCUP_MIRROR_SERVER_IP: $ROBOCUP_MIRROR_SERVER_IP"
echo "ROBOCUP_GAMECONTROLLER_IP: $ROBOCUP_GAMECONTROLLER_IP"
echo "X_POS: $X_POS"
echo "Y_POS: $Y_POS"
echo "ANGLE: $ANGLE"
echo "ROBOT_NAME: $ROBOT_NAME"
echo "COMPETITION: $COMPETITION"
echo "GOALIE: $GOALIE"
echo "TEAM_ID: $TEAM_ID"
echo "ENABLE_PYBULLET: $ENABLE_PYBULLET"

BRINGUP_DIR=$(rospack find soccerbot)
cat > $BRINGUP_DIR/config/game.yaml << EOF
ROBOCUP_ROBOT_ID: $ROBOCUP_ROBOT_ID
ROBOCUP_TEAM_COLOR: $ROBOCUP_TEAM_COLOR
ROBOCUP_SIMULATOR_ADDR: $ROBOCUP_SIMULATOR_ADDR
ROBOCUP_MIRROR_SERVER_IP: $ROBOCUP_MIRROR_SERVER_IP
ROBOCUP_GAMECONTROLLER_IP: $ROBOCUP_GAMECONTROLLER_IP
X_POS: $X_POS
Y_POS: $Y_POS
ANGLE: $ANGLE
ROBOT_NAME: $ROBOT_NAME
COMPETITION: $COMPETITION
GOALIE: $GOALIE
ENABLE_PYBULLET: $ENABLE_PYBULLET
TEAM_ID: $TEAM_ID
EOF
exec roslaunch  soccerbot soccerbot.launch fake_localization:=false robot_name:=$ROBOT_NAME x_pos:=$X_POS y_pos:=$Y_POS a_pos:=$ANGLE goalie:=$GOALIE competition:=$COMPETITION
