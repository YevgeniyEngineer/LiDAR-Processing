#ifndef LIDAR_PROCESSING__CLUSTERING_HPP
#define LIDAR_PROCESSING__CLUSTERING_HPP

// Containers
#include "queue.hpp"
#include "vector.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// STL
#include <cstdint>

namespace lidar_processing
{
using ClusteringLabel = std::uint32_t;

struct ClusteringConfiguration final
{
};

class Clusterer final
{
  public:
    Clusterer() = default;
    ~Clusterer() = default;

    void update_configuration(const ClusteringConfiguration &configuration);

    void reserve_memory(std::uint32_t number_of_points = 200'000U);

    template <typename PointT>
    void cluster(const pcl::PointCloud<PointT> &cloud_in, std::vector<ClusteringLabel> &labels);

  private:
};
} // namespace lidar_processing

#endif // LIDAR_PROCESSING__CLUSTERING_HPP
