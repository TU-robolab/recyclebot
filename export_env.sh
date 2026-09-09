#!/bin/bash

ENV_FILE=".env"

echo "USER_NAME=${USER}" > ${ENV_FILE}
echo "USER_ID=$(id -u $USER)" >> ${ENV_FILE}
echo "GROUP_NAME=$(id -gn $USER)" >> ${ENV_FILE}
echo "GROUP_ID=$(id -g $USER)" >> ${ENV_FILE}
echo "DISPLAY=${DISPLAY}" >> ${ENV_FILE}
echo "WAYLAND_DISPLAY=${WAYLAND_DISPLAY}" >> ${ENV_FILE}
echo "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}" >> ${ENV_FILE}
echo "REMOTE_HOSTNAME=ur" >> ${ENV_FILE}

# Robot addresses.
#
# REMOTE_IP is the generic fallback and the one docker-compose maps to the
# hostname "ur" via extra_hosts. It is correct for a cell running one arm at a
# time, including when arms are swapped onto the same address.
#
# The per-arm variables below take precedence when set (see
# recycle_bot/robot_profile.py: robot_ip). They matter once two arms are on the
# network simultaneously, since one REMOTE_IP cannot describe both. Leave them
# commented out unless that is your situation — the launches verify the
# connected robot's model against ur_type either way and refuse to drive the
# wrong arm.
echo "REMOTE_IP=${REMOTE_IP:-192.168.1.102}" >> ${ENV_FILE}
# echo "UR3E_ROBOT_IP=192.168.1.102" >> ${ENV_FILE}
# echo "UR16E_ROBOT_IP=192.168.1.103" >> ${ENV_FILE}

# Grant the container access to the X display.
#
# Without this, any GUI from the container (RViz, rqt, realsense-viewer) dies
# with "Error: Can't open display: :0". The container runs as a different user
# than the one owning the X session, so X refuses the connection.
#
# These grants live in the running X server, not in a file: they are wiped by
# every logout and reboot, which is why the error keeps coming back after a
# restart even though nothing in the repo changed. Applying them here means the
# `source ./export_env.sh` already required before `docker compose up` also
# repairs the display access, instead of it being something to remember.
#
# Scope: `+local:root` admits any local root process to the display, and the
# container's processes arrive as root from X's point of view. That is a
# deliberate, modest widening on a single-user workstation — it is not
# network-wide (`xhost +` would be, and is never appropriate). Revoke with:
#     xhost -local:root && xhost -SI:localuser:root
#
# Skipped when no display is present, so this stays usable over SSH and in CI.
if [ -n "${DISPLAY}" ] && command -v xhost >/dev/null 2>&1; then
    xhost +si:localuser:root >/dev/null 2>&1 \
        && echo "[export_env] X access granted to localuser:root" \
        || echo "[export_env] WARNING: xhost +si:localuser:root failed"
    xhost +local:root >/dev/null 2>&1 \
        || echo "[export_env] WARNING: xhost +local:root failed"
else
    echo "[export_env] no DISPLAY (or no xhost) — skipping X access grant;" \
         "GUI tools in the container will not open"
fi

# Export Docker build optimizations (shell environment only)
export DOCKER_BUILDKIT=1  # Enable BuildKit for cache mounts and faster builds
export COMPOSE_BAKE=true  # Enable Bake for efficient multi-service build orchestration