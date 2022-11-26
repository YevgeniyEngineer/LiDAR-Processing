cmake_minimum_required (VERSION 3.18)
project (lidar_processing)

set(CMAKE_CXX_STANDARD 17)

# cmake -DCMAKE_BUILD_TYPE=Release ..
set(CMAKE_BUILD_TYPE "release" CACHE INTERNAL "")

# Where executables will be stored
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(original_cmake_build_type ${CMAKE_BUILD_TYPE})

add_subdirectory(./lidar_processing/conversion)
add_subdirectory(./lidar_processing/data_reader_node)
add_subdirectory(./lidar_processing/transform_node)
add_subdirectory(./lidar_processing/ground_segmentation_node)

set(CMAKE_BUILD_TYPE "${original_cmake_build_type}" CACHE INTERNAL "")