ARG BASE_IMAGE=utrarobosoccer/noetic

FROM $BASE_IMAGE as dependencies
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
WORKDIR /root/src
RUN apt update && rosdep update --rosdistro noetic
ADD . .
RUN rosdep install --from-paths . --ignore-src -r -s  | grep 'apt-get install' | awk '{print $3}' | sort  >  /tmp/catkin_install_list
WORKDIR /root/dependencies

FROM $BASE_IMAGE as builder
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt update && \
    apt install -q -y software-properties-common && \
    add-apt-repository ppa:apt-fast/stable -y && \
    echo debconf apt-fast/maxdownloads string 16 | debconf-set-selections && \
    echo debconf apt-fast/dlflag boolean true | debconf-set-selections && \
    echo debconf apt-fast/aptmanager string apt-get | debconf-set-selections && \
    apt install -q -y apt-fast && \
    apt clean
RUN apt update && apt-fast install -y \
    screen \
    vim \
    python3-pip \
    python3-catkin-tools \
    python3-protobuf \
    protobuf-compiler \
    libprotobuf-dev \
    libjpeg9-dev \
    wget \
    ccache \
    dirmngr \
    gnupg2 \
    lsb-release \
    net-tools \
    iputils-ping \
    apt-utils \
    software-properties-common \
    sudo \
    unzip \
    ros-noetic-robot-state-publisher \
    curl \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-xkb1 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-render-util0 \
    libxcb-randr0 \
    libxcb-keysyms1 \
    libxcb-xinerama0 \
    qt5-default \
    qtbase5-dev \
    python3-pyqt5
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install keyboard-configuration # This needs to be its own individual step

# CUDA Installation
# Architecture: Use sbsa for arm build
# CUDA Installation Ref: https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_network
# CUDNN (Ref: https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#installlinux)
ARG ARCHITECTURE=x86_64
ARG OS=ubuntu2004
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/$OS/$ARCHITECTURE/cuda-$OS.pin && \
    sudo mv cuda-$OS.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/${OS}/$ARCHITECTURE/3bf863cc.pub && \
    sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/${OS}/$ARCHITECTURE/ /" && \
    sudo apt-get update
RUN DEBIAN_FRONTEND=noninteractive sudo apt-fast -yq --no-install-recommends install cuda libcudnn8 libcudnn8-dev libnccl2 libnccl-dev

RUN pip install --upgrade pip Cython pybullet

RUN curl -sSL https://get.docker.com/ | sh

COPY soccerbot/scripts/install_mxnet.sh install_mxnet.sh
RUN bash install_mxnet.sh

COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

COPY --from=dependencies /tmp/catkin_install_list /tmp/catkin_install_list
RUN apt update && apt-fast install -y $(cat /tmp/catkin_install_list)

# Create User
ARG USER="robosoccer"
RUN groupadd -g 1000 $USER && \
    useradd -u 1000 -g 1000 -mrs /bin/bash -b /home -p $(openssl passwd -1 $USER) $USER && \
    usermod -aG sudo $USER && \
    echo "$USER ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    usermod --append --groups 29,20,104,46,5,44 $USER

WORKDIR /home/$USER/catkin_ws
RUN chown -R $USER /home/$USER/catkin_ws
USER $USER

# Predownload yolo3 mobilenet
RUN curl https://apache-mxnet.s3-accelerate.dualstack.amazonaws.com/gluon/models/yolo3_mobilenet1.0_coco-66dbbae6.zip \
    --create-dirs --output /home/$USER/.mxnet/models/yolo3_mobilenet1.0_coco-66dbbae6.zip && \
    unzip /home/$USER/.mxnet/models/yolo3_mobilenet1.0_coco-66dbbae6.zip -d /home/$USER/.mxnet/models/

# Build C++ ROS Packages such as AMCL first
COPY --from=dependencies --chown=$USER /root/src/amcl src/soccerbot/amcl
RUN source /opt/ros/noetic/setup.bash && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN source /opt/ros/noetic/setup.bash && catkin build --no-status amcl
RUN rm -rf src/soccerbot

# Build Python ROS Packages
COPY --from=dependencies --chown=$USER /root/src src/soccerbot
RUN source /opt/ros/noetic/setup.bash && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
RUN source /opt/ros/noetic/setup.bash && catkin build --no-status soccerbot
RUN echo "source /home/$USER/catkin_ws/devel/setup.bash" >> ~/.bashrc

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/mxnet/
