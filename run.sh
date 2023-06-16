#!/bin/bash

xhost +local:docker
docker run -it --network host --rm \
    -e DISPLAY=${DISPLAY} \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --gpus all \
    -v /usr/lib/nvidia-container:/usr/lib/nvidia-container \
    -v /usr/bin/nvidia-container-runtime:/usr/bin/nvidia-container-runtime \
    -v /usr/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu \
    -v /lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu \
    -v /etc/machine-id:/etc/machine-id \
    -v /var/lib/dbus:/var/lib/dbus \
    -t flip-master:latest