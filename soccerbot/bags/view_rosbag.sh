#!/usr/bin/env bash

sudo chown -R $USER rosbag.bag.active
sudo chgrp -R $(id -g $USER) rosbag.bag.active

rosbag reindex -f rosbag.bag.active

roscore | ( sleep 1 && rqt_bag rosbag.bag.active )