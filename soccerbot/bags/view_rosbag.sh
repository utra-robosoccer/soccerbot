#!/usr/bin/env bash

sudo chown -R $USER robot1.bag.active
sudo chgrp -R $(id -g $USER) robot1.bag.active

rosbag reindex -f robot1.bag.active

roscore | ( sleep 1 && rqt_bag robot1.bag.active )