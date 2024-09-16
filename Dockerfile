ARG BASE_IMAGE=utrarobosoccer/noetic
ARG INSTALL_CUDA=true
ARG ARCHITECTURE=x86_64
ARG OS=ubuntu2004

FROM $BASE_IMAGE as dependencies
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-get update && \
    apt-get install -q -y software-properties-common && \
    add-apt-repository ppa:apt-fast/stable -y && \
    echo debconf apt-fast/maxdownloads string 16 | debconf-set-selections && \
    echo debconf apt-fast/dlflag boolean true | debconf-set-selections && \
    echo debconf apt-fast/aptmanager string apt-get | debconf-set-selections && \
    apt-get install -q -y apt-fast && \
    apt-get clean
RUN apt-get update && apt-fast install -y --no-install-recommends \
    screen \
    vim \
    python3-pip \
    python3-catkin-tools \
    python3-protobuf \
    protobuf-compiler \
    libprotobuf-dev \
    libjpeg8-dev  \
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
    python3-pyqt5 \
    python-is-python3 \
    git \
    python3-setuptools
RUN DEBIAN_FRONTEND=noninteractive apt-get -y --no-install-recommends install keyboard-configuration # This needs to be its own individual step

# CUDA Installation
# Architecture: Use sbsa for arm build
# CUDA Installation Ref: https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_network
# CUDNN (Ref: https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html#installlinux)
RUN if [[ "$INSTALL_CUDA" == "true" ]] ; then \
    wget --progress=dot:mega https://developer.download.nvidia.com/compute/cuda/repos/$OS/$ARCHITECTURE/cuda-$OS.pin && \
    mv cuda-$OS.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/${OS}/$ARCHITECTURE/3bf863cc.pub && \
    add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/${OS}/$ARCHITECTURE/ /" && \
    apt-get update; fi

RUN if [[ "$INSTALL_CUDA" == "true" ]] ; then DEBIAN_FRONTEND=noninteractive apt-fast -yq --no-install-recommends install cuda libcudnn9-cuda-12 libcudnn9-dev-cuda-12 libnccl2 libnccl-dev; fi

# TODO redo docker for jetson
RUN if [[ "$(dpkg --print-architecture)" == "arm64" ]] ; then \
    apt-get update && \
    apt-get install -y libopenblas-base libopenmpi-dev libomp-dev libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev && \
    wget https://developer.download.nvidia.cn/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl &&\
    pip3 install 'Cython<3' && \
    pip3 install numpy torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl && \
#    git clone --branch v0.16.1 https://github.com/pytorch/vision torchvision && \
#    cd torchvision && \
#    export BUILD_VERSION=0.16.1 && \
#    python3 setup.py install --user && \
#    cd ../ && \
#    pip install 'pillow<7' && \
    rm -rf torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl ; fi

# Create User
RUN groupadd -g 1000 robosoccer && \
    useradd -u 1000 -g 1000 -mrs /bin/bash -b /home -p $(openssl passwd -1 robosoccer) robosoccer && \
    usermod -aG sudo robosoccer && \
    echo "robosoccer ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    usermod --append --groups 29,20,104,46,5,44 robosoccer

# Allow nice for all users TODO research
RUN echo "*                -       priority        -20" >> /etc/security/limits.conf

# Install python dependencies
USER robosoccer
COPY requirements /tmp/requirements
ENV PATH=/home/robosoccer/.local/bin:$PATH
RUN pip install --upgrade pip
RUN pip install -r /tmp/requirements/requirements.txt
RUN if [[ "$INSTALL_CUDA" == "true" ]] ; then pip install -r /tmp/requirements/requirements-gpu.txt ; fi
RUN sudo apt-get update && rosdep update --rosdistro noetic  # TODO add an automated way
RUN apt-fast install -y --no-install-recommends $(cat /tmp/requirements/rosdep.txt)

FROM dependencies as builder

RUN mkdir -p /home/robosoccer/catkin_ws/src/soccerbot
WORKDIR /home/robosoccer/catkin_ws/src/soccerbot
ADD . .
RUN apt-fast install -y --no-install-recommends $(rosdep install --from-paths . --ignore-src -r -s  | grep 'apt-get install' | awk '{print $5}' | sort)

WORKDIR /home/robosoccer/catkin_ws

# Build Python ROS Packages
RUN source /opt/ros/noetic/setup.bash && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
RUN source /opt/ros/noetic/setup.bash && catkin build --no-status soccerbot
RUN echo "source /home/robosoccer/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN sudo ln -s /home/robosoccer/catkin_ws/devel/lib/python3/dist-packages/soccer_msgs  /opt/ros/noetic/lib/python3/dist-packages/
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda/targets/aarch64-linux/lib/:/usr/local/cuda-10.2/lib64
ENV PYTHONPATH=$PYTHONPATH:/home/robosoccer/.local/lib/python3.8/site-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/robosoccer/catkin_ws/devel/lib/python3/dist-packages
