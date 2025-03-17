#!/bin/bash
set -e

. "/opt/ros/${ROS_DISTRO}/setup.bash"
. "${ROS2_WS}/install/setup.bash"


# activate Python virtual environment (if it exists)
if [ -d "/venv" ]; then
    source /venv/bin/activate
fi

export PYTHONPATH="$PYTHONPATH:$(python -c 'import site; print(site.getsitepackages()[0])')"


exec "$@"