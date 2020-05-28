FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \
    apt-get purge --autoremove -y curl && \
rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 10.2.89
ENV CUDA_PKG_VERSION 10-2=$CUDA_VERSION-1

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-cudart-$CUDA_PKG_VERSION \
cuda-compat-10-2 && \
ln -s cuda-10.2 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.2 brand=tesla,driver>=384,driver<385 brand=tesla,driver>=396,driver<397 brand=tesla,driver>=410,driver<411 brand=tesla,driver>=418,driver<419"

# Install mesa
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y mesa-utils libgl1-mesa-dri


# Setup ROS
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Install ROS Melodic
RUN apt-get update \
    && apt install -y ros-noetic-desktop-full

# Install dev dependencies
RUN apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

RUN apt-get install -y python3-catkin-tools python3-rospkg


# Install python pip
RUN apt-get -y install \
    python3-pip \
    python3-dev \
    python3-catkin-pkg-modules \
    python3-yaml

# Install project dependencies
RUN apt-get -y install \
    ros-noetic-husky-gazebo

# ros-melodic-pointgrey-camera-description \
# ros-melodic-pointgrey-camera-driver \
# Install user utils
# ros-melodic-velodyne-* \
#ros-melodic-apriltag-ros \
# ros-melodic-rtabmap-ros \
# ros-melodic-teleop-twist-keyboard

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
