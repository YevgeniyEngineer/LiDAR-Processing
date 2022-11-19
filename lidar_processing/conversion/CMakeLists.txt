cmake_minimum_required (VERSION 3.18)

include(GNUInstallDirs)

project (conversion)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL 1.7.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(conversion 
    SHARED 
        ${CMAKE_CURRENT_SOURCE_DIR}/include/conversion.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/point_types.hpp
)
set_target_properties(conversion PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(conversion PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(
    conversion PUBLIC 
        cxx_std_17
)

add_library(CONVERSION::conversion ALIAS conversion)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)

install(TARGETS conversion
  EXPORT conversionConfig
  LIBRARY DESTINATION lib
)

export(TARGETS conversion
  NAMESPACE CONVERSION::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/conversionConfig.cmake"
)

install(EXPORT conversionConfig
  DESTINATION "${CMAKE_INSTALL_DATADIR}/CONVERSION/cmake"
  NAMESPACE CONVERSION::
)