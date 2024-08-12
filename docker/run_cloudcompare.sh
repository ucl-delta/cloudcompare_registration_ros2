#!/bin/bash

# Source environment
source /setup_cloudcompy310.sh

# Run Cloud Compare with XVFB in Background
Xvfb :0 -screen 0 1024x768x16 & 
CloudCompare 2>&1 /dev/null &
