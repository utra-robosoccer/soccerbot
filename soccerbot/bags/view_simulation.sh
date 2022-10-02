#!/usr/bin/env bash
# Simply run this file using ./view_simulation.sh to replay the last recorded webots session

# If coming from CI, simply copy the recording.json into the folder (../../external/webots/projects/samples/contests/robocup/controllers/referee/) and then run
# fix_recording_json.py and then run this file

google-chrome --disable-web-security --user-data-dir=.chrome --disable-fre ../../external/webots/projects/samples/contests/robocup/controllers/referee/recording.html
