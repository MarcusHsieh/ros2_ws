#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

echo "--- Setting up variables for Docker ---"

# Define paths for clarity
HOST_WS_PATH=~/ros2_ws
CONTAINER_WS_PATH=/ros2_ws
HOST_XML_PATH="${HOST_WS_PATH}/super_client.xml"
CONTAINER_XML_PATH="${CONTAINER_WS_PATH}/super_client.xml"

# --- Configuration ---
# <<< IMPORTANT: Set your Laptop's actual IP address here >>>
LAPTOP_IP="192.168.0.7"
ROS_DOMAIN_ID_TO_USE=30
DOCKER_IMAGE="dustynv/ros:humble-ros-core-l4t-r32.7.1"
# --- End Configuration ---

echo "Host Workspace: ${HOST_WS_PATH}"
echo "Container Workspace: ${CONTAINER_WS_PATH}"
echo "Host XML Config: ${HOST_XML_PATH}"
echo "Container XML Config: ${CONTAINER_XML_PATH}"
echo "Using Laptop IP: ${LAPTOP_IP} in XML (CHECK: ${HOST_XML_PATH} contains this IP!)"
echo "Using ROS_DOMAIN_ID: ${ROS_DOMAIN_ID_TO_USE}"
echo "Using Docker Image: ${DOCKER_IMAGE}"
echo "----------------------------------------"

# --- Pre-run Checks ---
if [ ! -d "${HOST_WS_PATH}" ]; then
  echo "Error: Host workspace directory '${HOST_WS_PATH}' not found."
  echo "Please create it first (e.g., mkdir -p ${HOST_WS_PATH}/src)"
  exit 1
fi

if [ ! -f "${HOST_XML_PATH}" ]; then
  echo "Error: Fast DDS XML config file '${HOST_XML_PATH}' not found."
  echo "Please create it first and ensure it points to the Laptop IP (${LAPTOP_IP})."
  exit 1
fi
echo "--- Pre-run checks passed ---"


echo "--- Launching Docker Container ---"
# Run the docker container
docker run -it --rm \
    --runtime nvidia \
    --net=host \
    -v "${HOST_WS_PATH}:${CONTAINER_WS_PATH}" \
    -v "${HOST_XML_PATH}:${CONTAINER_XML_PATH}" \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID_TO_USE} \
    # -e FASTRTPS_DEFAULT_PROFILES_FILE=${CONTAINER_XML_PATH} \
    ${DOCKER_IMAGE}

echo "--- Docker container exited ---"