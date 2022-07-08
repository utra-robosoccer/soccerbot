#!/usr/bin/env bash
# Simply run this file using ./view_simulation.sh to replay the last recorded webots session

google-chrome --disable-web-security --user-data-dir=.chrome --disable-fre ../../external/webots/projects/samples/contests/robocup/controllers/referee/recording.html
