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

# Export Docker build optimizations (shell environment only)
export DOCKER_BUILDKIT=1  # Enable BuildKit for cache mounts and faster builds
export COMPOSE_BAKE=true  # Enable Bake for efficient multi-service build orchestration