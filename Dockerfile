FROM ros:noetic as dependencies
WORKDIR /root/src
ADD . .
RUN rosdep update --rosdistro noetic && apt update && rosdep install --from-paths . --ignore-src -r -s  | grep 'apt-get install' | awk '{print $3}' | sort  >  /tmp/catkin_install_list
RUN mv requirements.txt /tmp/requirements.txt
WORKDIR /root/dependencies

FROM ros:noetic as builder
SHELL ["/bin/bash", "-c"]

# Dependencies
COPY --from=dependencies /tmp/catkin_install_list /tmp/catkin_install_list
COPY --from=dependencies /tmp/requirements.txt /tmp/requirements.txt
RUN apt-get update && apt-get install -y $(cat  /tmp/catkin_install_list)