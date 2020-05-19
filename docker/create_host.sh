#!/usr/bin/bash
docker run -it \
    -p 8888:8888  \
    -p 11345:11345  \
    --mount type=bind,source=/home/$USER/ros_docker,target=/home/ros \
    --gpus all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name ss \
    safer_stack

export containerId=$(docker ps -l -q)
xhost +local:'docker inspect --format='' $containerId'
docker start -a -i $containerId
