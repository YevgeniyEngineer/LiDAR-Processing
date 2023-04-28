cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(dbscan)

add_library(${PROJECT_NAME}
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/dbscan.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dbscan_nanoflann.hpp
)

set_target_properties(${PROJECT_NAME}
    PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/KDTree)

target_include_directories(${PROJECT_NAME}
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    INTERFACE
    kdtree::kdtree
)

add_library(dbscan::dbscan
    ALIAS
    ${PROJECT_NAME}
)

add_executable(example_dbscan
    ${CMAKE_CURRENT_SOURCE_DIR}/example_dbscan.cpp
)

target_link_libraries(example_dbscan
    PRIVATE
    dbscan::dbscan
)
