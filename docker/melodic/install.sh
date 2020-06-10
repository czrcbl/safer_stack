#! /usr/bin/bash
# Installation of conponents on user home folder.
# These are not included on the Dockerfile to ease the developtment,
# They can be moved there for deployment.
rosdep init
sudo rosdep fix-permissions 
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin config --init
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
catkin build
source ~/.bashrc


pip install mxnet-cu102 gluoncv
pip install gluoncv
# Python dependencies with pip
# pip install pyyaml

# pip3 install pyyaml \
    # rospkg \
    # numpy

# cd ~/catkin_ws/src
# in order to get oepncv bridge on python3
# git clone -b melodic https://github.com/ros-perception/vision_opencv.git


# Py3 Geometry
# git clone -b melodic-devel https://github.com/ros/geometry2
# git clone -b melodic-devel https://github.com/ros/geometry

# catkin build
# source ~/.bashrc

# Install Detectron2
# pip3 install torch torchvision opencv-python cython
# pip3 install -U 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'
# python3 -m pip install detectron2 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu102/index.html

