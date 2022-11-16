cmake_minimum_required (VERSION 3.18)
project (lidar_processing)
 
set(CMAKE_CXX_STANDARD 17)

# Where executables will be stored
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_subdirectory(lidar_processing/data_reader_node)
