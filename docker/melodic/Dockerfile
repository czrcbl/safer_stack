FROM nvidia/cudagl:10.2-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

# # setup timezone
# RUN echo 'Etc/UTC' > /etc/timezone && \
#     ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
#     apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# built-in packages
# RUN apt-get update \
#     && apt-get upgrade -y \
#     && apt-get install -y --no-install-recommends software-properties-common curl \
#     && apt-get install -y --no-install-recommends --allow-unauthenticated \
#         supervisor \
#         openssh-server pwgen sudo vim-tiny \
#         net-tools \
#         lxde x11vnc xvfb \
#         gtk2-engines-murrine ttf-ubuntu-font-family \
#         firefox \
#         nginx \
#         python-pip python-dev build-essential \
#         mesa-utils libgl1-mesa-dri \
#         gnome-themes-standard gtk2-engines-pixbuf gtk2-engines-murrine pinta arc-theme \
#         dbus-x11 x11-utils \
#         terminator \
#         dirmngr \
#         gnupg2 \
    # && apt-get autoclean \
    # && apt-get -y autoremove \
    # && rm -rf /var/lib/apt/lists/*

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

RUN apt-get install -y python-catkin-tools
RUN rosdep init

# Install python pip
RUN apt-get -y install python-pip python3-pip

# Install project dependencies
RUN apt-get -y install ros-melodic-pointgrey-camera-driver \
    ros-melodic-pointgrey-camera-description \
    ros-melodic-husky-gazebo \
    ros-melodic-velodyne-* \
    ros-melodic-apriltag-ros \ 
    python3-empy

# Install gzweb dependencies
# RUN apt-get install -y libjansson-dev libboost-dev imagemagick libtinyxml-dev cmake build-essential

# Add user
ENV USER=ros
ENV PASSWD=ros
ENV UID=1000
ENV GID=1000

RUN groupadd $USER && \
    useradd --create-home --no-log-init -g $USER $USER && \
    usermod -aG sudo $USER && \
    echo "$PASSWD:$PASSWD" | chpasswd && \
    chsh -s /bin/bash $USER && \
    # Replace 1000 with your user/group id
    usermod  --uid $UID $USER && \
    groupmod --gid $GID $USER

COPY post_install.sh /post_install.sh
WORKDIR /home/$USER
ENV HOME=/home/$USER \
    SHELL=/bin/bash

USER $USER
