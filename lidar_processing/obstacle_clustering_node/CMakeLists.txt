cmake_minimum_required (VERSION 3.18)

include(GNUInstallDirs)

project (obstacle_clustering_node)
 
set(CMAKE_CXX_STANDARD 17)

# colcon build --packages-select obstacle_clustering_node

find_package(PCL 1.7.2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/fast_euclidean_clustering
    ${PROJECT_SOURCE_DIR}/concave_hull
    ${PROJECT_SOURCE_DIR}/convex_hull
)

file(GLOB SOURCES
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
    "${PROJECT_SOURCE_DIR}/src/*.hpp"
    "${PROJECT_SOURCE_DIR}/include/*.hpp"
    "${PROJECT_SOURCE_DIR}/fast_euclidean_clustering/fast_euclidean_clustering.hpp"
    "${PROJECT_SOURCE_DIR}/concave_hull/concave_hull.hpp"
    "${PROJECT_SOURCE_DIR}/convex_hull/convex_hull.hpp"
    "${PROJECT_SOURCE_DIR}/convex_hull/convex_hull.cpp"
)

add_executable(${PROJECT_NAME} ${SOURCES})
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs geometry_msgs visualization_msgs)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} CONVERSION::conversion)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)