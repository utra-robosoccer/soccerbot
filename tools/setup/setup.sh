#!/usr/bin/env bash

check_internet_connection () {
    if ! ping -q -c 1 -W 1 google.com > /dev/null; then
        echo "No internet connection. Please check your internet connection to install the webots simulator."
        exit 1
    fi
}

ask_question() {
    while true; do
        read -p "$1 [Y/n]: " -n 1 -r response
        echo ""
        case $response in
            [Yy] | "") return 0;;
            [Nn]) return 1;;
            * ) echo "Please answer yes or no.";;
        esac
    done
}

setup_cuda(){
   if (( has_sudo )); then
      export OS=ubuntu2004
      export ARCHITECTURE=x86_64 # sbsa for ARM
      wget https://developer.download.nvidia.com/compute/cuda/repos/$OS/$ARCHITECTURE/cuda-$OS.pin
      sudo mv cuda-$OS.pin /etc/apt/preferences.d/cuda-repository-pin-600
      sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/${OS}/$ARCHITECTURE/3bf863cc.pub
      sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/${OS}/$ARCHITECTURE/ /"
      sudo apt-get update && sudo apt-get -y install cuda libcudnn9-cuda-12 libcudnn9-dev-cuda-12 libnccl2 libnccl-dev
   else
      echo "Please install CUDA manually!"
   fi

}


setup_ros(){
   if (( has_sudo )); then
        echo "Setting up ROS ..."
        sudo apt-get install -y python3-pip vim git git-lfs python-is-python3
        pip3 install pre-commit
        echo "export PATH=/home/$USER/.local/bin:$PATH" >> ~/.bashrc && source ~/.bashrc

        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt install -y curl # if you haven't already installed curl
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
        sudo apt update && sudo apt install -y ros-noetic-desktop-full
        echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && source ~/.bashrc
        sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
        sudo rosdep init
        rosdep update

        sudo sh \
            -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
                > /etc/apt/sources.list.d/ros-latest.list'
        wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
        sudo apt-get update && sudo apt-get install -y python3-catkin-tools


   else
        echo "Please install ROS manually!"
   fi

}

setup_repo(){
   if (( has_sudo )); then
        echo "Setting up Repo ..."
        mkdir -p ~/catkin_ws/src && cd ~/catkin_ws
        catkin init
        catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug # For Debug builds
        cd src
        git clone --recurse-submodules git@github.com:utra-robosoccer/soccerbot.git
        git pull
        git submodule sync
        git submodule update --init --recursive
        git-lfs pull
        git submodule foreach git-lfs pull

        cd ~/catkin_ws/src/soccerbot/
        sudo apt-get install -y $(cat tools/setup/rosdep.txt)
        pip install --upgrade pip
        pip install -U setuptools[core]
        pip install -r /tools/setup/requirements.txt
        if (( wants_cuda ))  ; then
          pip install -r /tools/setup/requirements-gpu.txt
        fi
        pre-commit install
        rosdep update && rosdep install --from-paths . --ignore-src -r -y

        # Building the webots controllers
        cd ~/catkin_ws/src/soccerbot/external/hlvs_webots
        pip install -r controllers/referee/requirements.txt
        sudo apt-get install protobuf-compiler libprotobuf-dev ant
        make

        # Building GameController
        cd ~/catkin_ws/src/soccerbot/external/GameController
        ant

        mkdir -p /home/$USER/.ros/config && cd /home/$USER/.ros/config
        ln -s /opt/ros/noetic/etc/ros/python_logging.conf

        catkin build soccerbot
        echo "source /home/$USER/catkin_ws/devel/setup.bash" >> ~/.bashrc && source ~/.bashrc

   else
        echo "Please install Repo manually!"
   fi

}




has_sudo=0
wants_cuda=0
if ask_question "Do you have sudo rights?"; then
    has_sudo=1
fi
if ask_question "Do you have a nvidia graphics card and want to install cuda"; then
    wants_cuda=1
fi

if (( ! has_sudo )); then
    echo "Because, you don't have sudo rights, ensure all necessary ROS packages are installed."
fi

check_internet_connection

if (( wants_cuda ))  ; then
  setup_cuda
fi

setup_ros
setup_repo
