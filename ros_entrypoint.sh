#!/bin/bash
set -e

# Error handler to prevent shell detachment
# error_handler() {
#     echo "❌ Error encountered in entrypoint script."
#     echo "🔄 Dropping into interactive shell instead of exiting..."
#     exec /bin/bash  # Keep the shell open for debugging
# }
# trap error_handler ERR  # Catch errors and invoke error_handler()

. "/opt/ros/${ROS_DISTRO}/setup.bash"
# third-party packages built at image time (serial, handeye_calibration)
if [ -f /opt/third_party/setup.bash ]; then
    . /opt/third_party/setup.bash
fi
if [ -f "${ROS2_WS}/install/setup.bash" ]; then
    . "${ROS2_WS}/install/setup.bash"
    echo "ROS2 and ROS2_WS environment setup completed."
else
    echo "ROS2 environment setup completed (workspace not yet built)."
fi

# activate Python virtual environment (if it exists)
# if [ -d "/venv" ]; then
#     source /venv/bin/activate
#     echo "Python virtual environment activated."
# else
#     echo "Warning: Python virtual environment (/venv) not found!"
# fi
# export PYTHONPATH="$PYTHONPATH:$(python -c 'import site; print(site.getsitepackages()[0])')"



exec "$@"

# # keep container running interactively, even if an error occurs
# if [ -z "$1" ]; then
#     echo "No command provided, starting an interactive shell..."
#     exec /bin/bash
# else
#     exec "$@"
# fi
