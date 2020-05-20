#! /usr/bin/bash
# Installation of conponents on user home folder.
# These are not included on the Dockerfile to ease the developtment,
# They can be moved there for deployment.

# Python dependencies with pip
pip install pyyaml \
    rospkg

# Install Detectron2
pip3 install torch torchvision \
    opencv-python \
    cython \
    rospkg

# in order to get oepncv bridge on python3
git clone -b melodic https://github.com/ros-perception/vision_opencv.git src/vision_opencv
catkin build cv_bridge
source ~/.bashrc

# Install Detectron2
pip3 install -U 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'
python3 -m pip install detectron2 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu102/index.html