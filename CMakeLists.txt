cmake_minimum_required (VERSION 3.18)
project (lidar_processing)

set(CMAKE_CXX_STANDARD 17)

# In the build dir: 
# cmake -DCMAKE_BUILD_TYPE=Release .. 
# cmake --build . --target all --config Release

# Where executables will be stored
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_subdirectory(./lidar_processing/conversion)
add_subdirectory(./lidar_processing/data_reader_node)
add_subdirectory(./lidar_processing/transform_node)
add_subdirectory(./lidar_processing/ground_segmentation_node)

