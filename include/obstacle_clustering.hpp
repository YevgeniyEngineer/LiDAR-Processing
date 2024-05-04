#ifndef OBSTACLE_CLUSTERING_HPP
#define OBSTACLE_CLUSTERING_HPP

// STL
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <numeric>
#include <vector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Clustering
#include "adaptive_dbscan_clustering.hpp"
#include "adaptive_euclidean_clustering.hpp"
#include "dbscan_clustering.hpp"
#include "fec_clustering.hpp"

#define DEBUG_CLUSTERING 0

namespace lidar_processing
{
enum class ClusteringAlgorithm
{
    DBSCAN,
    FAST_EUCLIDEAN_CLUSTERING,
    ADAPTIVE_EUCLIDEAN_CLUSTERING,
    ADAPTIVE_DBSCAN
};

class ObstacleClusterer
{
  public:
    ObstacleClusterer(float neighbour_radius_threshold = 0.5, float cluster_quality = 0.5,
                      std::uint32_t min_cluster_size = 3,
                      std::uint32_t max_cluster_size = std::numeric_limits<std::uint32_t>::max(),
                      ClusteringAlgorithm clustering_algorithm = ClusteringAlgorithm::DBSCAN);

    ~ObstacleClusterer();

    void clusterObstacles(const pcl::PointCloud<pcl::PointXYZRGBL> &obstacle_cloud,
                          std::vector<pcl::PointCloud<pcl::PointXYZ>> &clustered_cloud);

  private:
    float neighbour_radius_threshold_;
    float cluster_quality_;
    std::uint32_t min_cluster_size_;
    std::uint32_t max_cluster_size_;
    ClusteringAlgorithm clustering_algorithm_;

    std::unique_ptr<clustering::DBSCANClustering<float, 3>> dbscan_clusterer_;
};

} // namespace lidar_processing

#endif // OBSTACLE_CLUSTERING_HPP
