#!/bin/bash

# Jason: Super hack, build the simulink diagram and export into C code

roscore &
ROSCORE_PID=$!

matlab -nodesktop -r 'try; rtwbuild("soccer_control_generated"); catch; end; quit' || exit
rm -rf ../soccer_control_generated
mkdir ../soccer_control_generated
tar -C ../soccer_control_generated -xf soccer_control_generated.tgz
touch ../soccer_control_generated/*.cpp
rm -rf soccer_control_generated_ert_rtw
rm -rf build_ros_model.sh
rm -rf soccer_control_generated.tgz
rm -rf slprj

kill $ROSCORE_PID
