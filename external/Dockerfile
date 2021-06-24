FROM ros:noetic

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
    ros-noetic-robot-state-publisher \
    curl

WORKDIR /root
COPY GameController GameController
COPY webots webots
SHELL ["/bin/bash", "-c"]
RUN cd webots && \
   ./scripts/install/linux_compilation_dependencies.sh
RUN echo WEBOTS_HOME="/root/webots" >> ~/.bashrc && \
    cd webots && \
    make clean || echo "Completed" && \
    make -j10

RUN apt update && apt-get -y install ant protobuf-compiler libprotobuf-dev libjpeg9-dev && \
    cd GameController && \
    ant

RUN cd webots/projects/samples/contests/robocup/controllers/referee && \
    pip3 install numpy==1.20.2 && \
    pip3 install -r requirements.txt

RUN apt-get install -y protobuf-compiler libprotobuf-dev libjpeg9-dev && \
    cd webots/projects/samples/contests/robocup && \
    WEBOTS_HOME=/root/webots make clean && \
    WEBOTS_HOME=/root/webots make

RUN apt-get install -y mesa-utils

RUN pip3 install transforms3d
RUN apt install -y default-jre