#!/usr/bin/env bash

function cleanup {
  kill $(rosnode info controller_spawner | grep Pid | awk '{print $2}')
  killall gzclient
  killall gzserver
}

trap cleanup INT
sleep infinity