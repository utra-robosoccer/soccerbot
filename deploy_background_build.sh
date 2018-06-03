#!/bin/bash
#
# Copyright 2015-2016 The MathWorks, Inc.

BUILD_SCRIPT=$1
MODEL_NAME=$2
CATKIN_WS=$3

# Launch a background process and record return status to a file
"$BUILD_SCRIPT" "$MODEL_NAME".tgz "$CATKIN_WS" &> "$MODEL_NAME"_build.log &
pid=$!
wait $pid
echo $? > "$MODEL_NAME"_build.stat

exit 0

