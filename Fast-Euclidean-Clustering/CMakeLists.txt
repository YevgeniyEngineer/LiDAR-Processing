cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(fast_euclidean_clustering)

find_package(nanoflann REQUIRED)

add_library(${PROJECT_NAME}
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/fec_clustering.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fec_point_cloud.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/fec_union_find.hpp
)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

target_include_directories(${PROJECT_NAME}
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    INTERFACE
    nanoflann::nanoflann
)

add_library(fast_euclidean_clustering::fast_euclidean_clustering
    ALIAS
    ${PROJECT_NAME}
)

set(EXAMPLE_NAME example_fast_euclidean_clustering)
add_executable(${EXAMPLE_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/example_fast_euclidean_clustering.cpp)
target_link_libraries(${EXAMPLE_NAME}
    PRIVATE
    fast_euclidean_clustering::fast_euclidean_clustering
)