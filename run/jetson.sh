#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

echo "--- Setting up variables for Docker ---"

# Paths to mount the host workspace into the container
HOST_WS_PATH=~/ros2_ws
CONTAINER_WS_PATH=/ros2_ws

# --- Configuration ---
ROS_DOMAIN_ID_TO_USE=30
DOCKER_IMAGE="dustynv/ros:humble-ros-core-l4t-r32.7.1"
# --- End Configuration ---

echo "Host Workspace: ${HOST_WS_PATH}"
echo "Container Workspace: ${CONTAINER_WS_PATH}"
echo "Using ROS_DOMAIN_ID: ${ROS_DOMAIN_ID_TO_USE}"
echo "Using Docker Image: ${DOCKER_IMAGE}"
echo "----------------------------------------"

# --- Pre-run Checks ---
if [ ! -d "${HOST_WS_PATH}" ]; then
  echo "Error: Host workspace directory '${HOST_WS_PATH}' not found."
  echo "Please create it first (e.g., mkdir -p ${HOST_WS_PATH}/src)"
  exit 1
fi
echo "--- Pre-run checks passed ---"


echo "--- Launching Docker Container ---"
# Run the docker container
docker run -it --rm \
    --runtime nvidia \
    --net=host \
    -v "${HOST_WS_PATH}:${CONTAINER_WS_PATH}" \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID_TO_USE} \
    ${DOCKER_IMAGE}

echo "--- Docker container exited ---"