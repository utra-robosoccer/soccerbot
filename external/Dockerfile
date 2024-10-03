FROM nvidia/cudagl:11.4.2-base

ENV DEBIAN_FRONTEND=noninteractive
#TODO how should i split, should it be its own repo?. The dockerfile should be in hlvs_webots and we can probably rebuild when there is a puch on it
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

ARG WEBOTS_VERSION="2022b"
ARG WEBOTS_DOWNLOAD_URL="https://github.com/cyberbotics/webots/releases/download/R${WEBOTS_VERSION}/webots_${WEBOTS_VERSION}_amd64.deb"

RUN wget --no-verbose --show-progress "$WEBOTS_DOWNLOAD_URL" -O "/tmp/webots_${WEBOTS_VERSION}.deb" && sudo apt-get install "/tmp/webots_${WEBOTS_VERSION}.deb" -y

ENV WEBOTS_HOME /usr/local/webots
RUN echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib/controller"  >> ~/.bashrc
RUN echo "export PYTHONPATH=$PYTHONPATH:$WEBOTS_HOME/lib/controller/python38"  >> ~/.bashrc && source ~/.bashrc


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
    make clean && \
    make

RUN pip3 install pyyaml

ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV GAME_CONTROLLER_HOME /usr/local/GameController

SHELL ["/bin/bash", "-c"]
