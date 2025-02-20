#!/bin/bash

ENV_FILE=".env"

echo "USER_NAME=${USER}" > ${ENV_FILE}
echo "USER_ID=$(id -u $USER)" >> ${ENV_FILE}
echo "GROUP_NAME=$(id -gn $USER)" >> ${ENV_FILE}
echo "GROUP_ID=$(id -g $USER)" >> ${ENV_FILE}
echo "DISPLAY=${DISPLAY}" >> ${ENV_FILE}
echo "WAYLAND_DISPLAY=${WAYLAND_DISPLAY}" >> ${ENV_FILE}
echo "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}" >> ${ENV_FILE}