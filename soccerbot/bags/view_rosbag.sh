#!/usr/bin/env bash
# example: ./view_rosbag.sh red_robot2

source ~/.bashrc

BAG=${1:-red_robot1}

google-chrome --disable-web-security --user-data-dir=.chrome --disable-fre ../../external/webots/projects/samples/contests/robocup/controllers/referee/recording.html &

if [[ -f "$BAG.bag" ]]; then
    roscore | ( sleep 1 && rqt_bag $BAG.bag )
fi
if [[ -f "$BAG.bag.active" ]]; then
    sudo chown -R $USER $BAG.bag.active
    sudo chgrp -R $(id -g $USER) $BAG.bag.active

    rosbag reindex -f $BAG.bag.active

    roscore | ( sleep 1 && rqt_bag $BAG.bag.active )
fi
