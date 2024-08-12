#!/bin/bash

# Default values
container_name="cloudcompy310_build"
argument=""
entrypoint=""
mount=""

# Parse options
while getopts "n:a:em" opt; do
  case $opt in
    n) container_name="$OPTARG"
    ;;
    a) argument="$OPTARG"
    ;;
    e) entrypoint="--entrypoint bash"
    ;;
    m) mount="-v `pwd`/ros2_livox_cloudcompare:/ros2_ws/src/ros2_livox_cloudcompare"
    ;;
    \?) echo "Invalid option -$OPTARG" >&2
        exit 1
    ;;
  esac
done

docker run -it --rm \
    -v "$HOME/Datasets/G40_SCANS/":/Datasets \
    --net=host \
    --ipc=host \
    --pid=host \
    --env UID=$(id -u) \
    --env GID=$(id -g) \
    $entrypoint \
    $mount \
    "${container_name}":latest $argument
