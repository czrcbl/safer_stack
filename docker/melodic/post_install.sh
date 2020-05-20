#! /usr/bin/bash
# Set ups the catkin workspace and add the setup file to
# .bashrc.
sudo rosdep fix-permissions 
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --no-install
catkin config --init
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
