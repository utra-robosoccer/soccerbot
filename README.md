## Soccerbot

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Docker Image CI](https://github.com/utra-robosoccer/soccerbot/actions/workflows/docker_image.yml/badge.svg)](https://github.com/utra-robosoccer/soccerbot/actions/workflows/docker_image.yml)
[![Webots Docker Image CI](https://github.com/utra-robosoccer/soccerbot/actions/workflows/docker_image_webots.yml/badge.svg)](https://github.com/utra-robosoccer/soccerbot/actions/workflows/docker_image_webots.yml)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/utra-robosoccer/soccerbot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/utra-robosoccer/soccerbot/alerts/)
[![Documentation Status](https://readthedocs.org/projects/soccerbot/badge/?version=latest)](https://soccerbot.readthedocs.io/en/latest/?badge=latest)
[![Docker Image Size](https://badgen.net/docker/size/utrarobosoccer/soccerbot?icon=docker&label=image%20size)](https://hub.docker.com/r/utrarobosoccer/soccerbot/)
[![Docker Pulls](https://badgen.net/docker/pulls/utrarobosoccer/soccerbot?icon=docker&label=pulls)](https://hub.docker.com/r/utrarobosoccer/soccerbot/)

The initial efforts to form new team within UTRA began summer 2016. However, it was not until summer 2017 that we were well-defined and the creation of our robots from scratch began. Following some recruitment around that time, our team had a small number of members split across three subsystems: mechanical, electrical and software. Later that summer, the embedded subsystem (microcontroller software), emerged out of the electrical subsystem, and the control subsystem was formed to focus on bipedal locomotion.

By the end of 2017, our team grew to more than 20 members, all engineering undergraduates at the University of Toronto. Around that time, we were highly focused on achieving the basic requirements to qualify for the 2018 RoboCup competition in Montreal that upcoming summer. After several sleepless February nights, the video below was produced along with a concise paper.

##### Robot Model (Bez1)

![Bez robot model](https://media.githubusercontent.com/media/utra-robosoccer/soccerbot/master/docs/images/bez1/bez1_0.jpg)

##### Bez1 kicking in simulation

![Bez kicking Gif](https://github.com/utra-robosoccer/soccerbot/blob/master/docs/images/bez1/kick.gif?raw=true)

#### Join Us

https://github.com/utra-robosoccer/soccerbot/wiki

#### Getting Started

https://github.com/utra-robosoccer/soccerbot/wiki/Onboarding

#### Testing Motion on Robot

```bash
roslaunch soccerbot sensors.launch __ns:=robot1
```

cd ~/ros2_ws
source devel/setup.bash
pytest src/soccerbot/soccer_trajectories/src/soccer_trajectories/test_trajectory.py::TestTrajectory::test_fixed_angles_trajectories

pytest src/soccerbot/soccer_trajectories/src/soccer_trajectories/test_trajectory.py::TestTrajectory::test_getupfront_trajectories

pytest src/soccerbot/soccer_pycontrol/src/soccer_pycontrol/test_walking.py::TestWalking::test_walk_1_real_robot

cd ~/ros2_ws

```bash
source devel/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1 # TODO look into
pytest src/soccerbot/soccer_object_detection/src/soccer_object_detection/test_object_detection.py::TestObjectDetection::test_object_detection_node_cam
```

Torch https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

```bash
wget https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
sudo apt-get install python3-pip libopenblas-base libopenmpi-dev libomp-dev -y
pip3 install 'Cython<3'
pip3 install numpy torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
```

Torchvision

```bash
sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev -y
git clone --branch v0.16.1 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.16.1
python3 setup.py install --user
cd ../
pip install 'pillow<7'
```

Verify in python console

```bash
import torch
print(torch.__version__)
print('CUDA available: ' + str(torch.cuda.is_available()))
print('cuDNN version: ' + str(torch.backends.cudnn.version()))
a = torch.cuda.FloatTensor(2).zero_()
print('Tensor a = ' + str(a))
b = torch.randn(2).cuda()
print('Tensor b = ' + str(b))
c = a + b
print('Tensor c = ' + str(c))

import torchvision
print(torchvision.__version__)
```

sudo snap install blender --channel=3.3lts/stable --classic
https://github.com/dfki-ric/phobos/releases/tag/2.0.0
https://github.com/dfki-ric/phobos/commit/757d7b58b41240ea4aa54e20ddd1665072e6da21

rosrun xacro xacro -o bez2.urdf bez2.xacro

todo look into xrdp remote desktop
uv pip install -r tools/setup/requirements.txt --python $(which python) --prefix $(python -m site --user-base)

to compare
uv pip install -r tools/setup/requirements.txt --python $(which python) --prefix $(python -m site --user-base) --no-cache-dir --force-reinstall

colcon build --symlink-install --packages-up-to
source ~/ros2_ws/install/setup.bash
/home/manx52/ros2_ws/install/soccer_msgs/local/lib/python3.10/dist-packages

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib:/home/manx52/ros2_ws/build/soccer_msgs:/home/manx52/ros2_ws/install/soccer_msgs/local/lib/python3.10/dist-packages
pip3 install setuptools==61.0

sudo apt install ros-humble-imu-tools

sudo apt install ros-humble-foxglove-bridge
sudo apt install ros-humble-xacro

pip install -U placo
pip install -U scipy
pip install -U scikit-learn
pip install -U ultralytics
pip install -U matplotlib
pip install -U pybullet
pip install -U cmeel-octomap

ros2 run rqt_tf_tree rqt_tf_tree --force-discover

sudo apt install ros-humble-usb-cam

pip install -U pandas
pip install setuptools==71
