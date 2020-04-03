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
# git clone https://github.com/eric-wieser/ros_numpy
git clone https://github.com/ros-drivers/pointgrey_camera_driver
```

Install flycapture2 SDK, for bumblebee2 support:

First, install the dependencies:

```bash
sudo apt-get install libraw1394-11 libavcodec57 libavformat57 libswscale4 libswresample2 libavutil55 libgtkmm-2.4-1v5 libglademm-2.4-1v5 libgtkglextmm-x11-1.2-0v5 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libusb-1.0-0
```

You have to install `flycapture2`, download [HERE](https://flir.app.boxcn.net/v/Flycapture2SDK), extract the file and `cd` to the folder, then launch `install_flycapture.sh` script.

Then:
```bash
cd ~/catkin_ws
catkin build
```
