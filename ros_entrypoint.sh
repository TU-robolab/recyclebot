#!/bin/bash
set -e

# Source global ROS setup
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Source ROS2 workspace if it exists
if [ -f "${ROS2_WS}/install/setup.bash" ]; then
    source "${ROS2_WS}/install/setup.bash"
fi

# Ensure .bashrc is sourced for interactive shells
if [[ $- == *i* ]]; then
    source ~/.bashrc
fi

exec "$@"