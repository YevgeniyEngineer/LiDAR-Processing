cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(lidar_processing_pipeline)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.7.2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# build rules for executable "data_reader_node"
add_executable(data_reader_node src/data_reader_node.cpp)
ament_target_dependencies(data_reader_node rclcpp sensor_msgs)
target_link_libraries(data_reader_node ${PCL_LIBRARIES})
