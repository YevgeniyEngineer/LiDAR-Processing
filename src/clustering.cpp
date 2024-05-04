#include "clustering.hpp"

#include <chrono>

namespace lidar_processing
{
Clusterer::Clusterer() : configuration_{}
{
    reserve_memory();
}

void Clusterer::update_configuration(const ClusteringConfiguration &configuration)
{
    configuration_ = configuration;
}

void Clusterer::reserve_memory(std::uint32_t number_of_points)
{
    kdtree_.reserve(number_of_points);
    points_.reserve(number_of_points);
    neigh_.reserve(number_of_points);
    indices_.reserve(number_of_points);
    is_seed_set_.reserve(number_of_points);
    queue_.reserve(number_of_points);
}

template <typename PointT>
void Clusterer::cluster(const pcl::PointCloud<PointT> &cloud_in, std::vector<ClusteringLabel> &labels)
{
    labels.assign(cloud_in.size(), UNDEFINED);

    if (cloud_in.empty())
    {
        return;
    }

    points_.clear();
    points_.reserve(cloud_in.size());
    for (const auto &point : cloud_in.points)
    {
        points_.push_back({point.x, point.y, point.z});
    }

    kdtree_.rebuild(points_);

    ClusteringLabel label = 0U;

    is_seed_set_.resize(cloud_in.size());

    for (std::uint32_t i = 0U; i < cloud_in.size(); ++i)
    {
        if (labels[i] == UNDEFINED)
        {
            queue_.push(i);
            indices_.clear();

            std::fill(is_seed_set_.begin(), is_seed_set_.end(), false);
            is_seed_set_[i] = true;

            while (queue_.size() > 0U)
            {
                const auto j = queue_.front();
                queue_.pop();

                if (labels[j] == UNDEFINED)
                {
                    labels[j] = label;
                    is_seed_set_[j] = true;

                    indices_.push_back(j);

                    kdtree_.radius_search(points_[j], configuration_.dist_sqr, neigh_);

                    for (const auto &[k, dist] : neigh_)
                    {
                        if (is_seed_set_[k])
                        {
                            continue;
                        }

                        is_seed_set_[k] = true;

                        if (labels[k] == UNDEFINED)
                        {
                            queue_.push(k);
                        }
                    }
                }
            }

            if ((indices_.size() < configuration_.min_cluster_size) ||
                (indices_.size() > configuration_.max_cluster_size))
            {
                for (const auto &index : indices_)
                {
                    labels[index] = INVALID;
                }
            }
            else
            {
                ++label;
            }
        }
    }
}

template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, std::vector<ClusteringLabel> &labels);

template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, std::vector<ClusteringLabel> &labels);

template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZL> &cloud_in, std::vector<ClusteringLabel> &labels);

template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in,
                                 std::vector<ClusteringLabel> &labels);

template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZRGBL> &cloud_in,
                                 std::vector<ClusteringLabel> &labels);

} // namespace lidar_processing
