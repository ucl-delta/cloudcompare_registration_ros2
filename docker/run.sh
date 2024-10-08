#!/bin/bash

# Source environment
source /setup_cloudcompy310.sh

# Source ROS2
source "/opt/ros/$ROS_DISTRO/setup.bash" 
source "/ros2_ws/install/setup.bash"

(
    echo "Starting XVFB and Cloudcompare"
    ./run_cloudcompare.sh > /dev/null 2>&1 
)

ros2 run ros2_livox_cloudcompare ros2_livox_cloudcompare