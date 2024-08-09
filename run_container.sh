#!/bin/bash

docker run -it --rm \
    -v "$HOME/Datasets/G40_SCANS/":/Datasets \
    --net=host \
    --ipc=host \
    --pid=host \
    --env UID=$(id -u) \
    --env GID=$(id -g) \
    cloudcompy310_build:latest $1
