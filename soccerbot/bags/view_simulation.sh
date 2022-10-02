#!/usr/bin/env bash
# Simply run this file using ./view_simulation.sh to replay the last recorded webots session

if [ -f "recording.html" ]; then
    google-chrome --disable-web-security --user-data-dir=.chrome --disable-fre recording.html
else
    google-chrome --disable-web-security --user-data-dir=.chrome --disable-fre ../../external/webots/projects/samples/contests/robocup/controllers/referee/recording.html
fi
