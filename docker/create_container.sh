#!/usr/bin/bash
# Crete a container with the recomended options.
# Note that the /home/ros folder of the container will
# be mounted on /home/Docker/safer_stack on host
export containerName=ss
docker run -it \
    -p 8888:8888  \
    -p 11345:11345 \
    -p 8090:8080 \
    -p 5900:5900 \
    --mount type=bind,source=/home/$USER/Docker/safer_stack,target=/home/ros \
    --gpus all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name $containerName \
    safer_stack

containerId=$(docker inspect --format="{{.Id}}" $containerName)
# export containerId=$(docker ps -l -q)
xhost +local:'docker inspect --format='' $containerId'
docker start -a -i $containerId
