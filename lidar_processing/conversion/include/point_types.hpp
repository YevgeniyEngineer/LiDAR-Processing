#pragma once
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
    float intensity;
    std::uint32_t label;
    PCL_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
};
} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t,
                                                                                                       label, label))