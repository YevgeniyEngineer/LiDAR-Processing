#ifndef LIDAR_PROCESSING__CLUSTERING_HPP
#define LIDAR_PROCESSING__CLUSTERING_HPP

// Containers
#include "kdtree.hpp"
#include "queue.hpp"
#include "vector.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// STL
#include <cstdint>

namespace lidar_processing
{
using ClusteringLabel = std::int32_t;

struct ClusteringConfiguration final
{
    float distance_squared{0.18F};
    float cluster_quality{0.5F};
    std::uint32_t min_cluster_size{4U};
    std::uint32_t max_cluster_size{std::numeric_limits<std::uint32_t>::max()};
};

class Clusterer final
{
  public:
    static constexpr ClusteringLabel UNDEFINED{std::numeric_limits<std::int32_t>::lowest()};
    static constexpr ClusteringLabel INVALID{-1};

    Clusterer();
    ~Clusterer() = default;

    void update_configuration(const ClusteringConfiguration &configuration);

    void reserve_memory(std::uint32_t number_of_points = 200'000U);

    template <typename PointT>
    void cluster(const pcl::PointCloud<PointT> &cloud_in, std::vector<ClusteringLabel> &labels);

  private:
    ClusteringConfiguration configuration_;

    KDTree<float, 3> kdtree_;
    containers::Vector<Point<float, 3>> points_;
    containers::Vector<decltype(kdtree_)::RetT> neigh_;
    containers::Vector<std::uint32_t> indices_;
    std::vector<bool> removed_;
    containers::Queue<std::uint32_t> queue_;
};

extern template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                                        std::vector<ClusteringLabel> &labels);

extern template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                                        std::vector<ClusteringLabel> &labels);

extern template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZL> &cloud_in,
                                        std::vector<ClusteringLabel> &labels);

extern template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in,
                                        std::vector<ClusteringLabel> &labels);

extern template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZRGBL> &cloud_in,
                                        std::vector<ClusteringLabel> &labels);
} // namespace lidar_processing

#endif // LIDAR_PROCESSING__CLUSTERING_HPP
