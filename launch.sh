#!/bin/bash

# Make directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
BUILD_DIR="${SCRIPT_DIR}/build"
LAUCH_DIR="${BUILD_DIR}"

# Launch
unset GTK_PATH

cd "${LAUCH_DIR}"
./dataloader &
./processor &

cd "${SCRIPT_DIR}"
rviz2 -d "visualisation/rviz2_config.rviz" &

wait
