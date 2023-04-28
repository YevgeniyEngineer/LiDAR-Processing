#ifndef FAST_EUCLIDEAN_CLUSTERING_HPP
#define FAST_EUCLIDEAN_CLUSTERING_HPP

#include "fec_point_cloud.hpp"
#include "fec_union_find.hpp"

#include <nanoflann.hpp>

#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace clustering
{
template <typename CoordinateType, std::uint32_t number_of_dimensions> class FECClustering
{
    static_assert(std::is_floating_point<CoordinateType>::value,
                  "FECClustering only works with floating point precision points");

  public:
    using PointCloudT = FECPointCloud<CoordinateType, number_of_dimensions>;
    using KdTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<CoordinateType, PointCloudT>,
                                                       PointCloudT, number_of_dimensions>;
    using KdTreePtr = typename std::shared_ptr<KdTree>;
    using Indices = std::vector<std::uint32_t>;

    FECClustering() = delete;

    /// @brief Constructor of FECClustering object
    /// @param points Input point cloud compatible with nanoflann::KDTreeSingleIndexAdaptor
    explicit FECClustering(const PointCloudT &points)
        : cluster_tolerance_(0), max_cluster_size_(std::numeric_limits<std::uint32_t>::max()), min_cluster_size_(1),
          quality_(0), points_(points), kdtree_index_(3, points_, nanoflann::KDTreeSingleIndexAdaptorParams(10))
    {
        // Check if the number of points is sufficient for clustering
        if (points_.size() < 2)
        {
            return;
        }
    }

    CoordinateType clusterTolerance() const
    {
        return cluster_tolerance_;
    }
    void clusterTolerance(CoordinateType cluster_tolerance)
    {
        cluster_tolerance_ = cluster_tolerance;
    }
    std::uint32_t maxClusterSize() const
    {
        return max_cluster_size_;
    }
    void maxClusterSize(std::uint32_t max_cluster_size)
    {
        max_cluster_size_ = max_cluster_size;
    }
    std::uint32_t minClusterSize() const
    {
        return min_cluster_size_;
    }
    void minClusterSize(std::uint32_t min_cluster_size)
    {
        min_cluster_size_ = min_cluster_size;
    }
    CoordinateType quality() const
    {
        return quality_;
    }
    void quality(CoordinateType quality)
    {
        quality_ = quality;
    }

    const auto getClusterIndices() const
    {
        return cluster_indices_;
    }

    void formClusters()
    {
        // Clean old indices
        cluster_indices_.clear();

        // Check if the number of points is sufficient for clustering
        if (points_.size() < 2 || max_cluster_size_ == 0)
        {
            return;
        }

        const auto number_of_points = points_.size();
        const CoordinateType cluster_tolerance_squared = cluster_tolerance_ * cluster_tolerance_;
        const auto nn_distance_threshold =
            static_cast<CoordinateType>(std::pow((1.0 - quality_) * cluster_tolerance_, 2.0));

        std::vector<bool> removed(number_of_points, false);
        std::vector<std::pair<std::uint32_t, CoordinateType>> neighbours;

        std::queue<std::uint32_t> queue;

        FECUnionFind<std::uint32_t> uf(number_of_points);

        // Perform clustering
        for (std::uint32_t index = 0; index < number_of_points; ++index)
        {
            if (removed[index])
            {
                continue;
            }

            queue.push(index);

            while (!queue.empty())
            {
                const auto p = queue.front();
                queue.pop();

                if (removed[p])
                {
                    continue;
                }

                neighbours.clear();
                const auto number_of_neighbours = kdtree_index_.radiusSearch(&points_[p][0], cluster_tolerance_squared,
                                                                             neighbours, search_parameters_);

                for (std::uint32_t i = 0; i < number_of_neighbours; ++i)
                {
                    const auto q = neighbours[i].first;
                    if (removed[q])
                    {
                        continue;
                    }

                    uf.merge(index, q);

                    if (neighbours[i].second <= nn_distance_threshold)
                    {
                        removed[q] = true;
                    }
                    else
                    {
                        queue.push(q);
                    }
                }
            }
        }

        // Merge indices
        std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> temp_cluster_indices;
        for (std::uint32_t index = 0; index < number_of_points; ++index)
        {
            std::uint32_t root = uf.find(index);
            temp_cluster_indices[root].push_back(index);
        }

        // Remove small clusters and update cluster_indices_ with valid clusters
        cluster_indices_.clear();
        for (auto &cluster : temp_cluster_indices)
        {
            const auto cluster_size = cluster.second.size();
            if (cluster_size >= min_cluster_size_ && cluster_size <= max_cluster_size_)
            {
                cluster_indices_[cluster.first] = std::move(cluster.second);
            }
        }
    }

  private:
    CoordinateType cluster_tolerance_;
    std::uint32_t max_cluster_size_;
    std::uint32_t min_cluster_size_;
    CoordinateType quality_;
    PointCloudT points_;
    nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<CoordinateType, PointCloudT>, PointCloudT,
                                        number_of_dimensions>
        kdtree_index_;
    nanoflann::SearchParams search_parameters_;

    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> cluster_indices_;
};
} // namespace clustering

#endif // FAST_EUCLIDEAN_CLUSTERING_HPP