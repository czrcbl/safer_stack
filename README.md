# SAFER STACK

Safer Stack project repository.

## Installation

The project can be run in the provided docker container or be installed on a Ubuntu 18.04 machine.

In both installation scenarios, the `sand_mine` model has to be copied to `~/gazebo/models` folder.

### Docker Setup
The `docker` folder contain a Docker file for `melodic`, porting to `noetic` is planned.

Docker deployment is the preferred way.

Install [Docker](https://docs.docker.com/engine/install/).

Create the folder on host to mount the home folder on the container.  
Default: (edit `docker/create_container.sh` to change default):
```bash
mkdir -p ~/Docker/safer_stack/catkin_ws/src
cd ~/Docker/safer_stack/catkin_ws/src
# Clone this repository inside the src folder.
git clone https://github.com/czrcbl/safer_stack
```

Build the image
``` bash
cd /docker/{ros-distro}
docker build --tag safer_stack:melodic .
```

The Dockerfile creates a user already, called `ros` (password: `ros`).
In order to run the container run:
```bash
sh docker/create_container.sh
```

This scrips will share your display server with the container, so you can view Gazebo and Rviz from inside the container in your host machine screen.

Then, from inside the container:
```
cd ~/catkin_ws/src/safer_stack
sh docker/melodic/install.sh
```

To initialize the catkin workspace and install the python dependencies locally.

#### Managing the Container

To start the container and attach:
```
docker start -a -i ssm 
```

(`ssm` (safer stack melodic) is the default name for the container created in `create_container.sh`)

To start a new terminal on a container that was already started run:
```
docker exec -it ssm bash
```

### Normal Installation

In order to run the project on an Ubuntu18.04 machine, install the required packages: 

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

## Project Structure

The project follows a common layout of a `ROS` project:

* `src`: Contains the project python package called `safer_stack`;
* `nodes`: Contains the nodes in python (most) and C++, we tried to keep node code to a minimum while the functionalities reseides on the `safer_stack` package;
* `launch`: contains the launch files, they are divided in the following way;
	* `simulations`: contains the experiments;
	* `spawn`: contains the files responsible for spawning models into the simulation;
	* `SLAM`: contains launch files to start the SLAM algorithms, they can be run in conjunction to a experiment;
* `models`: contains some models used on the simulation, most obtained from the `Open Surce Robotics Fundation` repository;
* `urdf`: contain the modified models of the LIDAR and the Stereo Camera;

## Running the Code

### Simple Simulation

To start the simple simulation with LIDAR data:
``` bash
roslaunch safer_stack sim_lidar_nomap.launch
```

To move and control the truck based on the LIDAR data:
``` bash
rosrun safer_stack control_husky.py lidar
```

The robot should stop at 7 meters from the crest of the pile.

### Simulation with Rtabmap

In separate terminal run the following commands in the following order:
``` bash
roslaunch safer_stack sim_rtabmap_stereo.launch

roslaunch safer_stack rtabmap_stereo.launch

rosrun safer_stack distance_camera_ground.py 0

rosrun safer_stack control_husky.py rtabmap
```

To reset husky position and delete the map, run:
```bash
rosrun safer_stack reset_simulation.py
```


### Object Detection and Tracking
 
``` bash
roslaunch safer_stack sim_base_with_objects.launch

rosrun safer_stacl
```
  