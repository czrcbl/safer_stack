# SAFER STACK

## Instalation

```bash
cd ~/catkin_ws/src
git clone https://github.com/eric-wieser/ros_numpy
git clone https://github.com/ros-drivers/pointgrey_camera_driver
```

You have to install `flycapture2`, download [HERE](https://flir.app.boxcn.net/v/Flycapture2SDK), extract the file and `cd` to the folder, then launch `install_flycapture.sh` script.

Then:
```bash
cd ~/catkin_ws
catkin_make
```
