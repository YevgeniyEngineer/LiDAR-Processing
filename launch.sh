#!/bin/bash

# Make directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
BUILD_DIR="${SCRIPT_DIR}/build"
LAUCH_DIR="${BUILD_DIR}"

# Launch
unset GTK_PATH

cd "${LAUCH_DIR}"
./point_cloud_publisher_node &
./processing_node &

cd "${SCRIPT_DIR}"
rviz2 -d "rviz2_config/rviz2_config.rviz" &

wait