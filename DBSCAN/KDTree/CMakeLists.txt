cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(kdtree)

# Compillation instructions
# cd build
# cmake -DCMAKE_BUILD_TYPE=Release ..
# make
set(BUILD_TESTS True)

# Add library
find_package(TBB REQUIRED)

add_library(${PROJECT_NAME}
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/kdtree.hpp
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
    TBB::tbb
)

add_library(kdtree::kdtree
    ALIAS
    ${PROJECT_NAME}
)

# Build example
set(EXAMPLE_NAME example_kdtree)

add_executable(${EXAMPLE_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/example_kdtree.cpp
)

target_link_libraries(${EXAMPLE_NAME}
    PRIVATE
    kdtree::kdtree
)

# Build tests
if(BUILD_TESTS)
    find_package(GTest REQUIRED)

    enable_testing()

    set(TEST_NAME test_kdtree)

    include(GoogleTest)

    add_executable(${TEST_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/test_kdtree.cpp
    )

    target_link_libraries(${TEST_NAME}
        PRIVATE
        GTest::gtest_main
        kdtree::kdtree
    )

    gtest_discover_tests(${TEST_NAME})
endif()