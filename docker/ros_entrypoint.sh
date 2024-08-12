#!/bin/bash
set -e

# Create User for Shared Memory Transport
if ! id -u ros &>/dev/null; then
    useradd --uid ${UID:=1000} --create-home --shell /bin/bash ros
    chown -R ros:ros /ros2_ws
    echo "Created new user 'ros' and changed ownership of ros2_ws"
fi


# source /setup_cloudcompy310.sh

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

# Execute the command as the `ros` user
exec su ros -c "$@"
