# SAFER STACK

## Instalation

Required packages:  
`ros-melodic-husky-gazebo ros-melodic-velodyne-* ros-melodic-apriltag-ros python3-empy catkin build`


Workspace creation with `python3` support:

First, install some build dependencies:
```bash
sudo apt-get install python-catkin-tools python3-dev python3-pip
```

```bash
pip3 install catkin_pkg numpy rospkg opencv-python
```

Now configure the workspace with `python3`:
```bash
mkdir ~/catkin_ws
cd ~/catkin_ws
mkdir src
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
#catkin config --install
catkin config --no-install
```

Now clone the required packages:
```bash
cd ~/catkin_ws/src
git clone -b melodic https://github.com/ros-perception
```

Bumblebee2 support:

```bash
sudo apt-get install ros-melodic-pointgrey-camera-driver ros-melodic-pointgrey-camera-description
```

Then:
```bash
cd ~/catkin_ws
catkin build
```
