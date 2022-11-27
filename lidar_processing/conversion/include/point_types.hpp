#pragma once
#ifndef POINT_TYPES_HPP_
#define POINT_TYPES_HPP_

#define PCL_NO_PRECOMPILE

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/register_point_struct.h> // POINT_CLOUD_REGISTER_POINT_STRUCT

namespace pcl
{
// definition of the custom data type
// Example:: pcl::PointCloud<pcl::PointXYZIL>
struct EIGEN_ALIGN16 PointXYZIL // enforce SSE padding for correct memory alignment
{
    PCL_ADD_POINT4D; // preferred way of adding a XYZ+padding
    PCL_ADD_INTENSITY;
    std::uint32_t label;
    PCL_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
};

// definition of a custom data type pcl::PointXYZIIDX - x, y, z, intensity, idx
struct EIGEN_ALIGN16 PointXYZIIDX // enforce SSE padding for correct memory alignment
{
    PCL_ADD_POINT4D; // preferred way of adding a XYZ+padding
    PCL_ADD_INTENSITY;
    std::uint32_t index;          // stores an index of the original point cloud
    PCL_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
};

// definition of a custom data type pcl::PointXYZRGBI - x, y, z, intensity, color
struct EIGEN_ALIGN16 PointXYZRGBI // enforce SSE padding for correct memory alignment
{
    PCL_ADD_POINT4D; // preferred way of adding a XYZ+padding
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY;
    PCL_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
};

} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t,
                                                                                                       label, label))

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIIDX,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t,
                                                                                                       index, index))

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZRGBI,
                                  (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, intensity, intensity))

#endif // POINT_TYPES_HPP_