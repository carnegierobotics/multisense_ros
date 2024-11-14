ARG ros_codename=noetic

FROM ros:${ros_codename}-ros-base

ENV LANG=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /opt/crl/catkin_ws/src
WORKDIR /opt/crl/catkin_ws

COPY . /opt/crl/catkin_ws/src/multisense_ros

RUN . /opt/ros/noetic/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths src -y --ignore-src \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN . /opt/ros/noetic/setup.bash \
    && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
    && catkin_make test \
    && rm -r /opt/crl/catkin_ws/{src,devel,build}
