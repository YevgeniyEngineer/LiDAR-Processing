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

#include "polygonization.hpp"

namespace lidar_processing
{
void Polygonizer::update_configuration(const PolygonizationConfiguration &configuration)
{
    configuration_ = configuration;
}

void Polygonizer::reserve_memory(std::uint32_t max_points_per_cluster)
{
    polygon_.reserve(max_points_per_cluster);
    coordinates_xy_.reserve(max_points_per_cluster * 2U);
}

template <typename PointT>
void Polygonizer::polygonize(const pcl::PointCloud<PointT> &cloud, const std::vector<ClusteringLabel> &cluster_labels,
                             containers::Vector<containers::Vector<PointXYdZ>> &polygons)
{
}

template void Polygonizer::polygonize(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                      const std::vector<ClusteringLabel> &cluster_labels,
                                      containers::Vector<containers::Vector<PointXYdZ>> &polygons);

template void Polygonizer::polygonize(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                      const std::vector<ClusteringLabel> &cluster_labels,
                                      containers::Vector<containers::Vector<PointXYdZ>> &polygons);

template void Polygonizer::polygonize(const pcl::PointCloud<pcl::PointXYZL> &cloud,
                                      const std::vector<ClusteringLabel> &cluster_labels,
                                      containers::Vector<containers::Vector<PointXYdZ>> &polygons);

template void Polygonizer::polygonize(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                      const std::vector<ClusteringLabel> &cluster_labels,
                                      containers::Vector<containers::Vector<PointXYdZ>> &polygons);

template void Polygonizer::polygonize(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud,
                                      const std::vector<ClusteringLabel> &cluster_labels,
                                      containers::Vector<containers::Vector<PointXYdZ>> &polygons);
} // namespace lidar_processing
