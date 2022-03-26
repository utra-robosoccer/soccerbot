#!/usr/bin/env bash
# example: ./view_rosbag.sh red_robot2

sudo chown -R $USER $1.bag.active
sudo chgrp -R $(id -g $USER) $1.bag.active

rosbag reindex -f $1.bag.active

roscore | ( sleep 1 && rqt_bag $1.bag.active )