# SAFER STACK

Safer Stack project repository.

## Installation

The project can be run in the provided docker container or be installed on a Ubuntu18.04 machine.

In both installation scenarios, the `sand_mine` model has to be copied to `~/gazebo/models` folder.

## Docker Setup
The `docker` folder contain a Docker file for `melodic`, porting to `noetic` is planned.

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

The Dockerfile creates a user already, called `ros` (password: `ros`).
In order to run the container:
```bash
sh docker/create_container.sh
```

Run `sh /post_install.sh` inside the image to create the catkin workspace and add the setup files to `.bashrc` on the `ros` user.

Then run `install.sh` to install the python dependencies locally.

## Normal Installation

In order to run the project on an Ubuntu18.04 machie, install the required packages: 

Required packages: 
```bash
apt-get -y install ros-melodic-pointgrey-camera-driver \
    ros-melodic-pointgrey-camera-description \
    ros-melodic-husky-gazebo \
    ros-melodic-velodyne-* \
    ros-melodic-apriltag-ros \
    ros-melodic-teleop-twist-keyboard \
    ros-melodic-rtabmap-ros \
    python3-empy
```

Then:
```bash
cd ~/catkin_ws
catkin build
```
