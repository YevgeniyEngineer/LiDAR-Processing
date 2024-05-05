/*
 * Copyright (c) 2024 Yevgeniy Simonov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
