## Soccerbot Repository - For software running on the robot
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Travis build](https://travis-ci.org/utra-robosoccer/soccer_ws.svg?branch=master)](https://travis-ci.org/utra-robosoccer/soccer_ws)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/utra-robosoccer/soccer_ws.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/utra-robosoccer/soccer_ws/alerts/)
[![Coverity Scan Build Status](https://scan.coverity.com/projects/utra-robosoccer-soccer_ws/badge.svg)](https://scan.coverity.com/projects/utra-robosoccer-soccer_ws)

Welcome to the software repository, to start working on the robot, follow the instructions to install ros

http://wiki.ros.org/ROS/Installation

#### Prerequisites

Debian packages needed for robots (sudo apt-get install)
- git
- git-lfs
- python-catkin-tools
- net-tools
- indicator-ip

#### Setting up your IDE
- Use Jetbrains installer (https://www.jetbrains.com/toolbox/app/)
- Follow the CLion Setup here, use method 2 to add bash to the launch file https://github.com/ethz-asl/programming_guidelines/wiki/CLion
- In CLion, once you finish following the instructions, you should be able to reload CMake to have code hinting enabled
- Install the *.launch file plugins if you want to. Look up duckietown/hatchery from the third party repositories in Preferences/Plugins
- Add the python2.7 intepretor to CLion to get Clion code hinting. In Settings/Build,Execution,Deployment/Python Intepretor, add the system intepretor /usr/bin/python 2.7
- For debugging processes follow the steps here https://www.jetbrains.com/help/clion/attaching-to-local-process.html

#### Initialization of the code
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
sudo apt-get install python-catkin-tools # If you don't have the package installed yet.
catkin_init_workspace
git clone --recurse-submodules https://github.com/utra-robosoccer/soccer_ws #  To clone the repository
git lfs init
git lfs pull
cd soccer_ws
git checkout initials_branchname  # TO create a new branch, use git checkout -b initials_branchname
cd ~/catkin_ws
```
#### Installing submodules and dependencies
```
cd ~/catkin_ws/src/soccer_ws
git submodule update --recursive --init
sudo rosdep init # If first time using ROS in your environment.
rosdep update
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic # To install all dependencies (use correct ROS distro version), add --os ubuntu:xenial if your linux is based on it but has different distro name and version. Ubuntu 16.04 uses kinetic instead of melodic. For Jetson TX2 use kinetic.
```

#### Building the code
```
catkin build soccerbot # Use catkin clean to start with a clean build
source devel/setup.bash # Needs to be done everytime you finish building
```

Build and run tests
```
catkin build <pkg name> --verbose --catkin-make-args run_tests
```

#### Launching the robot
You should be ready to go now. Before running, setup your CLion IDE (above),  To run the robot:

```bash
roslaunch soccerbot soccerbot.launch
```

For simulation you can just run this

```bash
roslaunch soccerbot soccerbot.launch simulation:=true
```
