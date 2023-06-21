#!/bin/bash

xhost +local:docker
docker run -it --network host --rm \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    --device=/dev/dri \
    -t flip-master:latest