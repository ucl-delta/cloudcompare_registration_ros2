#!/bin/bash
set -e

# Creates User for Shared Memory Transport
id -u ros &>/dev/null || adduser --quiet --disabled-password --gecos '' --uid ${UID:=1000} ros

source /setup_cloudcompy310.sh

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
exec "$@"
