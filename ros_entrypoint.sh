#!/bin/bash
set -e

. "/opt/ros/${ROS_DISTRO}/setup.bash"
. "${ROS2_WS}/install/setup.bash"
. "alias ros='ros2'"

# activate Python virtual environment (if it exists)
if [ -d "/venv" ]; then
    source /venv/bin/activate
fi

#source ~/.bashrc  # alias ros='ros2'

exec "$@"