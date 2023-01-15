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

RUN pip3 install transforms3d

WORKDIR /usr/local

COPY webots/scripts webots/scripts
RUN ./webots/scripts/install/linux_compilation_dependencies.sh
RUN ./webots/scripts/install/linux_optional_compilation_dependencies.sh

COPY webots webots
RUN echo WEBOTS_HOME="/usr/local/webots" >> ~/.bashrc && \
    cd webots && \
    make

COPY GameController GameController
RUN apt update && apt-fast -y install ant protobuf-compiler libprotobuf-dev libjpeg9-dev && \
    cd GameController && \
    ant

COPY hlvs_webots hlvs_webots
RUN cd hlvs_webots/controllers/referee && \
    pip3 install numpy==1.20.2 && \
    pip3 install -r requirements.txt

RUN apt-get install -y protobuf-compiler libprotobuf-dev libjpeg9-dev && \
    cd hlvs_webots && \
    WEBOTS_HOME=/usr/local/webots make clean && \
    WEBOTS_HOME=/usr/local/webots make

RUN pip3 install pyyaml

ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV WEBOTS_HOME /usr/local/webots
ENV GAME_CONTROLLER_HOME /usr/local/GameController

SHELL ["/bin/bash", "-c"]
