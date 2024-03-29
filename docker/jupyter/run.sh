#!/bin/bash

xhost +           

docker run \
    --gpus all \
    -it \
    --runtime nvidia \
    -e DISPLAY=$DISPLAY \
    --device=/dev/video0 \
    --device=/dev/video1 \
    -v /lib/modules:/lib/modules \
    --cap-add SYS_PTRACE \
    -v /tmp:/tmp \
    l4t-humble-jupyter:latest /bin/bash