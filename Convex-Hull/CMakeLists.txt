cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(convex_hull)

set(ADD_PYTHON_BINDING TRUE)

# Compillation instructions
# cd build
# cmake -DCMAKE_BUILD_TYPE=Release ..
# make

# Dynamic library
add_library(${PROJECT_NAME}
    INTERFACE
    convex_hull.hpp
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
add_library(convex_hull::convex_hull
    ALIAS
    ${PROJECT_NAME}
)

# Test
add_executable(test_convex_hull test_convex_hull.cpp)
target_link_libraries(test_convex_hull
    PRIVATE
    convex_hull::convex_hull
)