#! /usr/bin/bash
sudo rosdep fix-permissions 
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin config --no-install
catkin config --init
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install gzserver
cd ~/
curl -sL https://raw.githubusercontent.com/creationix/nvm/v0.33.11/install.sh -o install_nvm.sh
bash install_nvm.sh
source ~/.bashrc
nvm install 8.11.1
nvm use 8.11.1
git clone https://github.com/osrf/gzweb
cd ~/gzweb
npm run deploy ---