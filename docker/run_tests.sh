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

cd /ros2_ws

colcon test --packages-select ros2_livox_cloudcompare \
            --event-handlers console_cohesion+  \
            --pytest-args 
