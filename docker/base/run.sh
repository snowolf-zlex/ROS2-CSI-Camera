#!/bin/bash

xhost +           

docker run \
    -it \
    --network host \
    --runtime nvidia \
    -e DISPLAY=$DISPLAY \
    --device=/dev/video0 \
    --device=/dev/video1 \
    -v /lib/modules:/lib/modules \
    -v /tmp:/tmp \
    --cap-add SYS_PTRACE \
    l4t_humble_base:latest /bin/bash
