#!/bin/bash

# Make directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
BUILD_DIR="${SCRIPT_DIR}/build"
LAUCH_DIR="${BUILD_DIR}/lidar_processing_pipeline"

if [ ! -d "${BUILD_DIR}" ]; then
    mkdir -p "${BUILD_DIR}"
fi

# Build
cd "${BUILD_DIR}"
cmake -DCMAKE_BUILD_TYPE=Release .. 
cmake --build . --target all --config Release