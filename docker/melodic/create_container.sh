#!/usr/bin/bash
# Crete a container with the recomended options.
# Note that the ~/Docker/safer_stack on host will
# be mounted on /home/ros folder on the container
rosVersion=melodic
containerName=ssm
imageName=safer_stack:melodic
docker create -it \
    -p 8888:8888  \
    --mount type=bind,source=/home/$USER/Docker/safer_stack,target=/home/ros \
    --gpus all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name $containerName \
    $imageName

containerId=$(docker inspect --format="{{.Id}}" $containerName)
# export containerId=$(docker ps -l -q)
xhost +local:'docker inspect --format='' $containerId'
docker start -a -i $containerId
