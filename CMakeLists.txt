# cmake -DCMAKE_BUILD_TYPE=Release ..
# cmake --build . --target all --config Release

cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(lidar_processing)

set(BUILD_TESTS True)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(TBB REQUIRED)
find_package(PCL 1.7.2 REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_subdirectory(./Convex-Hull)
add_subdirectory(./Concave-Hull)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# build rules for executable dataloader
add_executable(dataloader src/dataloader.cpp)
ament_target_dependencies(dataloader rclcpp sensor_msgs)
target_link_libraries(dataloader ${PCL_LIBRARIES} ${rclcpp_LIBRARIES})

# build rules for executable processor
add_executable(processor
    src/segmentation.cpp
    src/clustering.cpp
    src/polygonization.cpp
    src/processor.cpp
    src/conversions.cpp
    src/polygon_simplification.cpp
)
target_include_directories(processor
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${rclcpp_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
ament_target_dependencies(processor
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    builtin_interfaces
)
target_link_libraries(processor
    TBB::tbb
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${rclcpp_LIBRARIES}
    convex_hull::convex_hull
    concave_hull::concave_hull
)

if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()
    set(TEST_NAME test_all)
    include(GoogleTest)
    add_executable(${TEST_NAME}
        test/test_kdtree.cpp
        test/test_pointcloud.cpp
        test/test_cartesian_vector_2d_adapter.cpp
        test/test_cartesian_vector_3d_adapter.cpp
    )
    target_include_directories(${TEST_NAME}
        PRIVATE
            include
            src
    )
    target_link_libraries(${TEST_NAME}
        PRIVATE
            GTest::gtest_main
    )
    gtest_discover_tests(${TEST_NAME})
endif()
