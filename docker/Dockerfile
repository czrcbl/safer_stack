FROM nvidia/cudagl:10.2-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update
# RUN apt install -y nvidia-driver-440
RUN apt-get install -y mesa-utils

# Install sudo
RUN apt-get update && \
      apt-get -y install sudo

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


# Setup ROS

USER root

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
# RUN apt-get update && apt-get install --no-install-recommends -y \
#     python-rosdep \
#     python-rosinstall \
#     python-vcstools \
#     && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Install ROS Melodic
RUN apt-get update
RUN apt install -y ros-melodic-desktop-full

# Install dev dependencies
RUN apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
RUN apt-get install -y python-catkin-tools
RUN rosdep init

# Install python3 pip
RUN apt-get -y install python3-pip

# Install project dependencies
RUN apt-get -y install ros-melodic-pointgrey-camera-driver \
    ros-melodic-pointgrey-camera-description \
    ros-melodic-husky-gazebo \
    ros-melodic-velodyne-* \
    ros-melodic-apriltag-ros \ 
    python3-empy

# Install gzweb dependencies
RUN apt-get install -y libjansson-dev libboost-dev imagemagick libtinyxml-dev cmake build-essential

COPY post_install.sh /post_install.sh

USER $USER
WORKDIR /home/$USER
# Setup environment
# USER $USER
# RUN rosdep fix-permissions && rosdep update
# RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
# RUN /bin/bash -c "source ~/.bashrc"

# Create catkin workspace
# RUN mkdir -p /home/ros/catkin_ws/src
# RUN cd ~/catkin_ws/
# WORKDIR  /home/ros/catkin_ws
# RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
# RUN /bin/bash -c "source ~/.bashrc"