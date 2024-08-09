#!/bin/bash

# Source environment
source /setup_cloudcompy310.sh

# Source ROS2
source "/opt/ros/$ROS_DISTRO/setup.bash" 
source "/ros2_ws/install/setup.bash"

# Run Cloud Compare with XVFB in Background
Xvfb :0 -screen 0 1024x768x16 & 
CloudCompare 2>&1 /dev/null &

ros2 run ros2_livox_cloudcompare ros2_livox_cloudcompare