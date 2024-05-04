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

#include "clustering.hpp"

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
    removed_.reserve(number_of_points);
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
    removed_.assign(cloud_in.size(), false);

    const auto distance_squared_threshold =
        std::pow(1.0 - configuration_.cluster_quality, 2) * configuration_.distance_squared;

    ClusteringLabel label = 0U;
    for (std::uint32_t i = 0U; i < cloud_in.size(); ++i)
    {
        if (removed_[i])
        {
            continue;
        }

        queue_.push(i);
        indices_.clear();

        while (queue_.size() > 0U)
        {
            const auto j = queue_.front();
            queue_.pop();

            if (removed_[j])
            {
                continue;
            }

            kdtree_.radius_search(points_[j], configuration_.distance_squared, neigh_);

            for (const auto &[k, dist] : neigh_)
            {
                if (removed_[k])
                {
                    continue;
                }

                labels[k] = label;
                indices_.push_back(k);

                if (dist <= distance_squared_threshold)
                {
                    removed_[k] = true;
                }
                else
                {
                    queue_.push(k);
                }
            }
        }

        if ((indices_.size() < configuration_.min_cluster_size) || (indices_.size() > configuration_.max_cluster_size))
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

template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, std::vector<ClusteringLabel> &labels);

template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, std::vector<ClusteringLabel> &labels);

template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZL> &cloud_in, std::vector<ClusteringLabel> &labels);

template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in,
                                 std::vector<ClusteringLabel> &labels);

template void Clusterer::cluster(const pcl::PointCloud<pcl::PointXYZRGBL> &cloud_in,
                                 std::vector<ClusteringLabel> &labels);

} // namespace lidar_processing
