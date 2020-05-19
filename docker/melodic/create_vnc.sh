#!/usr/bin/bash
docker run -it \
    -p 8888:8888  \
    -p 11345:11345 \
    -p 8090:8080 \
    -p 6080:80 \
    -p 5900:5900 \
    --mount type=bind,source=/home/$USER/Docker/safer_stack,target=/root \
    --gpus all \
    --name ss \
    safer_stack

export containerId=$(docker ps -l -q)
xhost +local:'docker inspect --format='' $containerId'
docker start -a -i $containerId

su ros