FROM nvidia/cudagl:10.2-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y mesa-utils libgl1-mesa-dri

# Setup ROS
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Install ROS Melodic
RUN apt-get update
RUN apt install -y ros-melodic-desktop-full

# Install dev dependencies
RUN apt-get install -y \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential

RUN apt-get install -y python-catkin-tools python-rospkg


# Install python pip
RUN apt-get -y install python-pip \
    python3-pip \
    python3-dev \
    python3-catkin-pkg-modules \
    python3-yaml

# Install project dependencies
RUN apt-get -y install ros-melodic-pointgrey-camera-driver \
    ros-melodic-pointgrey-camera-description \
    ros-melodic-husky-gazebo \
    ros-melodic-velodyne-* \
    ros-melodic-apriltag-ros \
    ros-melodic-teleop-twist-keyboard \
    ros-melodic-rtabmap-ros \
    ros-melodic-move-base \
    python3-empy

# Install user utils
RUN apt install -y gedit \
    nano

# Install Visual Studio Code For development
# RUN apt-get install -y software-properties-common apt-transport-https wget
# RUN wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | apt-key add -
# RUN add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
# RUN apt-get update \
#   && apt install -y code

# Add user
ENV USER=ros \
    PASSWD=ros \
    UID=1000 \
    GID=1000

RUN groupadd $USER && \
    useradd --create-home --no-log-init -g $USER $USER && \
    usermod -aG sudo $USER && \
    echo "$PASSWD:$PASSWD" | chpasswd && \
    chsh -s /bin/bash $USER && \
    # Replace 1000 with your user/group id
    usermod  --uid $UID $USER && \
    groupmod --gid $GID $USER

# COPY post_install.sh /post_install.sh
WORKDIR /home/$USER
ENV HOME=/home/$USER \
    SHELL=/bin/bash

USER $USER
