#!/bin/bash
set -e

. "/opt/ros/${ROS_DISTRO}/setup.bash"
. "${ROS2_WS}/install/setup.bash"


# activate Python virtual environment (if it exists)
if [ -d "/venv" ]; then
    source /venv/bin/activate
fi


exec "$@"