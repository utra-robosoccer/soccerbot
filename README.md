## Soccerbot Repository - For software running on the robot
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Travis build](https://travis-ci.org/utra-robosoccer/soccer_ws.svg?branch=master)](https://travis-ci.org/utra-robosoccer/soccer_ws)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/utra-robosoccer/soccer_ws.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/utra-robosoccer/soccer_ws/alerts/)
[![Coverity Scan Build Status](https://scan.coverity.com/projects/utra-robosoccer-soccer_ws/badge.svg)](https://scan.coverity.com/projects/utra-robosoccer-soccer_ws)

Welcome to the software repository, to start working on the robot, follow the instructions to install ros

#### Prerequisites
First you need Ubuntu 18.04. Either obtain it using a Virtual Machine or Dual Boot your PC to Ubuntu and Windows  

The recommendation is to Dual Boot or have an entire computer dedicated to Ubuntu 18.04 because Robot Software is quite CPU/GPU heavy and you need a lot of system resources to run them.

For Dual Boot - https://opensource.com/article/18/5/dual-boot-linux  
For Virtual Machine
- VM Emulator https://www.virtualbox.org/wiki/Downloads
- Ubuntu Desktop https://ubuntu.com/download/desktop
- Make sure when you setup the Virtual Machine, move the .iso into the virtual machine files and also enable more CPU for the virtual machine. 50% of your CPU is good

Next, install ROS Melodic (for Ubuntu 18.04)
http://wiki.ros.org/ROS/Installation

Next, install these Debian packages
```
sudo apt-get install git git-lfs python-catkin-tools net-tools vim htop meshlab
```

[Install Nvidia CUDA Toolkit following the instructions here](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1804&target_type=debnetwork)  
(This is only for people with NVidia enabled GPUs for simulation and not on VM)
```
Operating System - Linux  
Architecture - x86  
Distribution - Ubuntu  
Version - 18.06  
Installer type - deb [network]  
```

On Nvidia CUDA involved computers. Follow the instructions carefully here to install CUDA
https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1804&target_type=debnetwork

#### Initialization of the code
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --recurse-submodules https://github.com/utra-robosoccer/soccer_ws #  To clone the repository
cd soccer_ws # To get into the local repository and perform git lfs commands
git lfs pull
git checkout branch_name  # TO create a new branch, use git checkout -b initials_branchname
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug # For Debug builds
```
#### Updating submodules and dependencies
```bash
# Updating Submodules
cd ~/catkin_ws/src/soccer_ws
git submodule update --recursive --init

# Updating Dependencies
cd ~/catkin_ws/
sudo rosdep init # Only need to do this once
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro melodic # To install all dependencies (use correct ROS distro version), add --os ubuntu:xenial if your linux is based on it but has different distro name and version. Ubuntu 16.04 uses kinetic instead of melodic. For Jetson TX2 use kinetic.
```

#### Setting up your IDE (CLion)
- Get the Jetbrains student membership (https://www.jetbrains.com/student/)
- Download Jetbrains installer (https://www.jetbrains.com/toolbox/app/) and install CLion

##### CLion
- Open Jetbains installer and install CLion
- Add shell run from IDE (This process might need to be redone everytime Jetbrain updates your Clion so come back to this step
```bash
gedit ~/.local/share/applications/jetbrains-clion.desktop
# Add the bash -i -c to the beginning of this line (rest of the line remain the same)
Exec=bash -i -c "/home/vuwij/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/192.7142.39/bin/clion.sh" %f
```
- Add a CMakelist file
```bash
cd ~/catkin_ws/src
catkin_init_workspace
```
- Open CLion,navigate to ~/catkin_ws/src/CMakelists.txt and click open as project
- On the bottom CMake bar, go to Cmake settings and add this line to Environment
```bash
ROS_PACKAGE_PATH=/home/vuwij/catkin_ws/src
```
- Change you IDE to light mode. Go to File > Settings > Appearance and change Theme to IntellJ
- Install the *.launch file plugins. Look up duckietown/hatchery from the third party repositories in Preferences/Plugins
- Add the python2.7 intepretor to CLion to get Clion code hinting. In Settings/Build,Execution,Deployment/Python Intepretor, Click the gear and add the "System Intepreter" /usr/bin/python2.7
- Debugging
  - Follow the steps here to setup your debugging https://www.jetbrains.com/help/clion/attaching-to-local-process.html
  - ```cd catkin_ws && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug # For Debug builds (you might need to rebuild```
  - Run > Attach to Process > Select the Process you want to attach to, you might want to rosnode info <node> to identify it's PID. Add a breakpoint

##### Pycharm Professional
- Open Jetbains installer and install Pycharm
- Add shell run from IDE (This process might need to be redone everytime Jetbrain updates your Clion so come back to this step
```bash
gedit ~/.local/share/applications/jetbrains-pycharm.desktop
# Add the bash -i -c to the beginning of this line (rest of the line remain the same)
Exec=bash -i -c "/home/vuwij/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/192.7142.39/bin/clion.sh" %f
```
- Open Pycharm, go to File > Open and open the repository name (soccer_ws)
- Change the Intepreter to python 2.7. Go to Project Interpreter > Add > System Intepreter and choose /usr/bin/python2
- Change you IDE to light mode. Go to File > Settings > Appearance and change Theme to IntellJ
- Install the *.launch file plugins. Look up duckietown/hatchery from the third party repositories in Preferences/Plugins
- Debugging
  - Run > Attach to Process > Select the Process you want to attach to, you might want to rosnode info <node> to identify it's PID. Add a breakpoint

##### Matlab
- Create a Mathworks account
- Download Matlab (https://www.mathworks.com/downloads/)
  - sudo run the install.run script (Might need to chmod +x the script)
  - Matlab license information can be found here (https://github.com/utra-robosoccer/soccer_ws/wiki/Onboarding)
  - Install all toolboxes if possible
- In matlab navigate to the the folder soccer_ws/soccer_control
- Execute any script (NOTE: Do not double click a folder to enter it, instead click on + to expand the folder when you are navigating). A starter script can be found in /test/walking

#### Setting up your IDE (Pycharm)
- Get the Jetbrains student membership (https://www.jetbrains.com/student/)
- Use Jetbrains installer (https://www.jetbrains.com/toolbox/app/) and install CLion and Pycharm Professional
- Add shell run from IDE (This process might need to be redone everytime Jetbrain updates your Clion so come back to this step
```bash
gedit ~/.local/share/applications/jetbrains-pycharm.desktop
Change the Exec line to this 
Exec=bash -i -c "/home/vuwij/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/192.7142.39/bin/clion.sh" %f
```
- Install the *.launch file plugins. Look up duckietown/hatchery from the third party repositories in Preferences/Plugins
- Debugging
  - Simply go to Attach to Process and lookup the node. You can find the node by identifying the PID using rosnode info
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
