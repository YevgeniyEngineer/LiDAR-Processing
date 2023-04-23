# cmake -DCMAKE_BUILD_TYPE=Release ..
# cmake --build . --target all --config Release

cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(lidar_processing_pipeline)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(TBB REQUIRED)
find_package(PCL 1.7.2 REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_subdirectory(./DBSCAN)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# build rules for executable "data_reader_node"
add_executable(point_cloud_publisher_node
    src/point_cloud_publisher_node.cpp
)
ament_target_dependencies(point_cloud_publisher_node
    rclcpp
    sensor_msgs
)
target_link_libraries(point_cloud_publisher_node
    ${PCL_LIBRARIES}
    ${rclcpp_LIBRARIES}
)

# build rules for executable "processing_node"
add_executable(processing_node
    src/processing_node.cpp
    src/ground_segmentation.cpp
)
target_include_directories(processing_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    third_party/fast_euclidean_clustering
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(processing_node
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    builtin_interfaces
)
target_link_libraries(processing_node
    TBB::tbb
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${rclcpp_LIBRARIES}
    dbscan::dbscan
)
