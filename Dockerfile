FROM ros:noetic as dependencies
WORKDIR /root/src
ADD . .
RUN rosdep update --rosdistro noetic && apt update && rosdep install --from-paths . --ignore-src -r -s  | grep 'apt-get install' | awk '{print $3}' | sort  >  /tmp/catkin_install_list
RUN mv requirements.txt /tmp/requirements.txt
WORKDIR /root/dependencies

FROM ros:noetic as builder
SHELL ["/bin/bash", "-c"]

# Install dependencies
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
    curl
COPY --from=dependencies /tmp/catkin_install_list /tmp/catkin_install_list
COPY --from=dependencies /tmp/requirements.txt /tmp/requirements.txt
RUN apt-get update && apt-fast install -y $(cat  /tmp/catkin_install_list)
RUN pip install -r /tmp/requirements.txt

# Build
WORKDIR /root/catkin_ws
COPY --from=dependencies /root/src src
RUN source /opt/ros/noetic/setup.bash && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
RUN source /opt/ros/noetic/setup.bash && catkin build soccerbot
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc