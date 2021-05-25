## Soccerbot Repository - For software running on the robot
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Travis build](https://travis-ci.org/utra-robosoccer/soccerbot.svg?branch=master)](https://travis-ci.org/utra-robosoccer/soccerbot)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/utra-robosoccer/soccerbot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/utra-robosoccer/soccerbot/alerts/)
[![Coverity Scan Build Status](https://scan.coverity.com/projects/utra-robosoccer-soccerbot/badge.svg)](https://scan.coverity.com/projects/utra-robosoccer-soccerbot)

Welcome to the software repository, to start working on the robot, follow the instructions to build and run the code

#### Building the code
```bash
cd ~/catkin_ws # As long as your current path is in a subdirectory of this folder
catkin build soccerbot # Use catkin clean to start with a clean build
source devel/setup.bash # Needs to be done everytime you finish building a new package
```

Build and run tests
```bash
catkin build <pkg name> --verbose --catkin-make-args run_tests
```

#### Launching the robot
You should be ready to go now. Before running, setup your CLion IDE (above),  To run the robot:

```bash
roslaunch soccerbot soccerbot_multi.launch simulation:=false multi:=false
```
- This file launches a soccerbot.lauunch which is a single robot. Each of these have multiple modules (localization, navigation) which are launch files for certain components of a robot
- Note that the arguments := are optional and the default ones are set in the launch files
test
