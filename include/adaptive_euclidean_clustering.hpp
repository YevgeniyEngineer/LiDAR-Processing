#ifndef ADAPTIVE_EUCLIDEAN_CLUSTERING_HPP
#define ADAPTIVE_EUCLIDEAN_CLUSTERING_HPP

#include <cstdint>
#include <vector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
void formAdaptiveEuclideanCluster(
    const pcl::PointCloud<pcl::PointXYZRGBL>& obstacle_cloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>>& clustered_cloud,
    float incremental_cluster_tolerance_m = 0.1,
    std::uint32_t min_cluster_size = 2U,
    std::uint32_t max_cluster_size = std::numeric_limits<std::uint32_t>::max(),
    float cluster_quality = 0.5f);
}

#endif // ADAPTIVE_EUCLIDEAN_CLUSTERING_HPP