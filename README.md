## Soccerbot Repository - For software running on the robot
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Travis build](https://travis-ci.org/utra-robosoccer/soccer_ws.svg?branch=master)](https://travis-ci.org/utra-robosoccer/soccer_ws)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/utra-robosoccer/soccer_ws.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/utra-robosoccer/soccer_ws/alerts/)
[![Coverity Scan Build Status](https://scan.coverity.com/projects/utra-robosoccer-soccer_ws/badge.svg)](https://scan.coverity.com/projects/utra-robosoccer-soccer_ws)

Welcome to the software repository, to start working on the robot, follow the instructions to install ros

http://wiki.ros.org/ROS/Installation

#### Prerequisites

Debian packages needed for robots (sudo apt-get install)
```bash
sudo apt-get install -y git vim net-tools python-catkin-tools
```

#### Initialization of the code
```bash
# Initialize the workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

# Cloning and checking out the repository
git clone --recurse-submodules https://github.com/utra-robosoccer/soccer_ws #  To clone the repository
cd soccer_ws
git checkout initials_branchname  # TO create a new branch, use git checkout -b initials_branchname

# Install GIT LFS
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install

# Updating submodules and dependencies
sudo rosdep init # If first time using ROS in your environment.
rosdep update
git submodule update --recursive --init
rosdep install --from-paths $PWD --ignore-src -r -y --rosdistro melodic # To install all dependencies (use correct ROS distro version), add --os ubuntu:xenial if your linux is based on it but has different distro name and version. Ubuntu 16.04 uses kinetic instead of melodic. For Jetson TX2 use kinetic.

# Installing Soccer control center
source ~/catkin_ws/devel/setup.bash
roscd soccer_control_center/scripts
./install.sh
```

#### Setting up your IDE
- Use Jetbrains installer (https://www.jetbrains.com/toolbox/app/)
- Follow the CLion Setup here, use method 2 to add bash to the launch file https://github.com/ethz-asl/programming_guidelines/wiki/CLion
- Rename jetbrains-clion.desktop to clion.desktop. This way Jetbrains toolbox doesn't override the file when you restart.
- In CLion, once you finish following the instructions, you should be able to reload CMake to have code hinting enabled
- Install the *.launch file plugins if you want to. Look up duckietown/hatchery from the third party repositories in Preferences/Plugins
- Add the python2.7 intepretor to CLion to get Clion code hinting. In Settings/Build,Execution,Deployment/Python Intepretor, add the system intepretor /usr/bin/python 2.7
- For debugging processes follow the steps here https://www.jetbrains.com/help/clion/attaching-to-local-process.html
- For convenience, there is a Clion Settings Repository for Soccer. Go to File > Settings > Settings Respository and add this as a read-only source https://github.com/utra-robosoccer/soccer-ws-clion-settings

#### Building the code
```bash
catkin build soccerbot # Use catkin clean to start with a clean build
source ~/catkin_ws/devel/setup.bash # Needs to be done everytime you finish building
```

#### Connecting the Robot
Edit your .bashrc, 
- it should look like this, but you have to run ifconfig to see the correct interface for your Wifi (replace wlp110s0)

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_IP=$(ifconfig $(ifconfig -a | sed 's/[ \t].*//;/^$/d' | tail -1 | sed 's/.$//') | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')
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

To keep the robot midair

```bash
roslaunch soccerbot soccerbot_simulation.launch frozen:=true
```

For omnibot, just run the omnibot launch file, replace robot.launch with simulation.launch for simulation

```bash
roslaunch soccerbot omnibot.launch dryrun:=true
```

For running in a mode where the hardware is not connected and you need to examine the output of what would be sent to hardware if the hardware was connected.

```bash
roslaunch soccerbot omnibot.launch dryrun:=false
```
