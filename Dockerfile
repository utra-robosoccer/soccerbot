# ROS 1 and ROS 2 simulation setup

FROM ros:kinetic
MAINTAINER  Jason (jiashen.wang@robotics.utias.utoronto.ca)

RUN apt-get update && apt-get install -q -y \
    bash-completion \
    dirmngr \
    git \
    gnupg2 \
    libasio-dev \
    libtinyxml2-dev \
    lsb-release \
    python3-pip \
    wget \
    curl \
    ssh \
    git \
    vim \
    unzip \
    supervisor \
    flex \
    bison \
    build-essential \
    lcov \
    checkinstall \
    apt-transport-https \
    libreadline-gplv2-dev \
    libncursesw5-dev \
    libssl-dev \
    libffi-dev \
    libsqlite3-dev \
    libgdbm-dev \
    libbz2-dev \
    libeigen3-dev \
    libusb-dev \
    protobuf-c-compiler \
    protobuf-compiler \
    software-properties-common \
    cmake-curses-gui \
    clang-3.8 \
    python-scipy \
    python-dev \
    python-mock \
    python-kombu \
    python-pip \
    doxygen \
    lsof \
    nano \
    liblz4-tool \
    libjs-mathjax \
    fonts-mathjax \
    net-tools \
    openssh-server \
    lighttpd \
    systemd

COPY . /home/utra/catkin_ws

# Run Core
CMD ["roscore"]
