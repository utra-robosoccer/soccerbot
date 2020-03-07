#!/bin/bash

if ! command -v matlab > /dev/null
then
    echo "Matlab not installed, exiting"
    exit 0;
fi

source ~/catkin_ws/devel/setup.bash
roscd soccer_control

matlab -nodesktop -r 'try; run_from_bash=1; main; catch; end; quit' || exit