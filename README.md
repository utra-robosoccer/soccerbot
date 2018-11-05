## Soccerbot Repository - For software running on the robot

[![Travis build](https://travis-ci.org/utra-robosoccer/soccer_ws.svg?branch=master)](https://travis-ci.org/utra-robosoccer/soccer_ws)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/utra-robosoccer/soccer_ws.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/utra-robosoccer/soccer_ws/alerts/)

Welcome to the software repository, to start working on the robot, follow the instructions to install ros

http://wiki.ros.org/ROS/Installation

#### Prerequisites

Debian packages needed for robots (sudo apt-get install)
- git
- git-gui
- python-catkin-tools
- vim
- net-tools
- indicator-ip

#### Setting up your IDE
- Use Jetbrains installer (https://www.jetbrains.com/toolbox/app/)
- Follow the CLion Setup here, use method 2 to add bash to the launch file https://github.com/ethz-asl/programming_guidelines/wiki/CLion
- Rename jetbrains-clion.desktop to clion.desktop. This way Jetbrains toolbox doesn't override the file when you restart.
- In CLion, once you finish following the instructions, you should be able to reload CMake to have code hinting enabled
- Install the *.launch file plugins if you want to. Look up duckietown/hatchery from the third party repositories in Preferences/Plugins
- Add the python2.7 intepretor to CLion to get Clion code hinting. In Settings/Build,Execution,Deployment/Python Intepretor, add the system intepretor /usr/bin/python 2.7

#### Initialization of the code
```bash
cd ~/catkin_ws/src
catkin_init_workspace
git clone --recurse-submodules https://github.com/utra-robosoccer/soccer_ws #  To clone the repository
cd soccer_ws
git checkout initials_branchname  # TO create a new branch, use git checkout -b initials_branchname
cd ~/catkin_ws
```
#### Installing submodules and dependencies
```
git submodule update --recursive --init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic # To install all dependencies (use correct ROS distro version), add --os ubuntu:xenial if your linux is based on it but has different distro name and version.
```

#### Building the code
```
catkin build soccerbot # Use catkin clean to start with a clean build
source devel/setup.bash # Needs to be done everytime you finish building
```

#### Connecting the Robot
Edit your .bashrc, 
- it should look like this, but you have to run ifconfig to see the correct interface for your Wifi (replace wlp110s0)
- Remember to have the correct distro (melodic or kinetic)

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
MY_IP=$(ifconfig wlp110s0 | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')
export ROS_IP=$MY_IP
export ROS_MASTER_URI=http://$ROS_IP:11311
```
#### Launching the robot
You should be ready to go now. Before running, setup your CLion IDE (above),  To run the robot:

```bash
roslaunch soccerbot soccerbot.launch
```

For simulation you can just run this

```bash
roslaunch soccerbot soccerbot_simulation.launch
```

For omnibot, just run the omnibot launch file, replace robot.launch with simulation.launch for simulation

```bash
roslaunch soccerbot omnibot.launch
```
