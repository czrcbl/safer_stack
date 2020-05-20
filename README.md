# SAFER STACK

# Docker Setup
The `docker` folder contain a Docker file for `melodic` and `noetic` distribution.
Docker deployment is the preferred way.

Install [Docker](https://docs.docker.com/engine/install/).

Create the folder on host to mount the home folder on the container.  
Default (edit `docker/create_container.sh` to change default):
```bash
mkdir -p ~/Docker/safer_stack/catkin_ws/src
```
Clone this repository inside the src folder.

Build the image
``` bash
cd /docker/{ros-distro}
docker build --tag safer_stack .
```
In order to run the container:
```bash
sh docker/create_container.sh
```
Note that in order to display the graphical interfaces on host, a method that may be unsafe is used by this script.

Run `sh /post_install.sh` to create the catkin workspace and add the setup files to .bashrc on the `ros` user.

Then run `install.sh` to install the python dependencies locally.




## Installation

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
