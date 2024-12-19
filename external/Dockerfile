FROM nvidia/cudagl:11.4.2-base

ENV DEBIAN_FRONTEND=noninteractive

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
    python3-protobuf \
    protobuf-compiler \
    libprotobuf-dev \
    libjpeg9-dev \
    wget \
    ant \
    dirmngr \
    gnupg2 \
    lsb-release \
    net-tools \
    iputils-ping \
    apt-utils \
    software-properties-common \
    sudo \
    curl \
    libxdamage1 \
    mesa-utils \
    default-jre \
    xvfb

RUN DEBIAN_FRONTEND=noninteractive apt-get -y --no-install-recommends install keyboard-configuration # This needs to be its own individual step

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic

COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

SHELL ["/bin/bash", "-c"]

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list && \
    apt-get update && apt-get purge -y '*opencv*' && apt-fast install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    ros-noetic-ros-base=1.5.0-1* \
    ros-noetic-robot=1.5.0-1* \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-pip \
    python3-catkin-tools \
    python3-protobuf \
    protobuf-compiler \
    libprotobuf-dev \
    libprotoc-dev \
    && rm -rf /var/lib/apt/lists/* && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

RUN groupadd -g 1000 robosoccer && \
    useradd -u 1000 -g 1000 -mrs /bin/bash -b /home -p $(openssl passwd -1 robosoccer) robosoccer && \
    usermod -aG sudo robosoccer && \
    echo "robosoccer ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    usermod --append --groups 29,20,104,46,5,44 robosoccer

USER robosoccer
ENV PATH=/home/robosoccer/.local/bin:$PATH
#TODO how should i split, should it be its own repo?. The dockerfile should be in hlvs_webots and we can probably rebuild when there is a puch on it


RUN pip3 install transforms3d

ARG WEBOTS_VERSION="2022b"
ARG WEBOTS_DOWNLOAD_URL="https://github.com/cyberbotics/webots/releases/download/R${WEBOTS_VERSION}/webots_${WEBOTS_VERSION}_amd64.deb"

RUN wget --no-verbose --show-progress "$WEBOTS_DOWNLOAD_URL" -O "/tmp/webots_${WEBOTS_VERSION}.deb"
RUN sudo apt update &&  sudo apt-fast -y install "/tmp/webots_${WEBOTS_VERSION}.deb"

ENV WEBOTS_HOME /usr/local/webots
RUN echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib/controller"  >> ~/.bashrc
RUN echo "export PYTHONPATH=$PYTHONPATH:$WEBOTS_HOME/lib/controller/python38"  >> ~/.bashrc

RUN mkdir -p /home/robosoccer/catkin_ws/src/soccerbot/external
WORKDIR /home/robosoccer/catkin_ws/src/soccerbot/external

COPY --chown=robosoccer hlvs_webots hlvs_webots
RUN cd hlvs_webots/controllers/referee && \
    pip3 install numpy==1.20.2 && \
    pip3 install -r requirements.txt

COPY --chown=robosoccer GameController GameController
RUN cd GameController && ant


RUN cd hlvs_webots && \
    make clean && \
    make


WORKDIR /home/robosoccer/catkin_ws/src/soccerbot/external
RUN sudo apt-get update && rosdep update --rosdistro $ROS_DISTRO
RUN apt-fast install -y --no-install-recommends $(rosdep install --from-paths . --ignore-src -r -s  | grep 'apt-get install' | awk '{print $5}' | sort)

RUN pip3 install protobuf==3.20.2
WORKDIR /home/robosoccer/catkin_ws
RUN source /opt/ros/noetic/setup.bash && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
RUN source /opt/ros/noetic/setup.bash && catkin build --no-status soccer_webots
RUN echo "source /opt/ros/noetic/lib/python3/dist-packages" >> ~/.bashrc


RUN pip3 install pyyaml


ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV GAME_CONTROLLER_HOME /home/robosoccer/catkin_wssrc/soccerbot/external/GameController/

SHELL ["/bin/bash", "-c"]
