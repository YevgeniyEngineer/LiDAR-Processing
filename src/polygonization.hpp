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

#ifndef LIDAR_PROCESSING__POLYGONIZATION_HPP
#define LIDAR_PROCESSING__POLYGONIZATION_HPP

#include "clustering.hpp"
#include "vector.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
struct PointXYdZ final
{
    double x{0.0F};
    double y{0.0F};
    float z_min{0.0F};
    float z_max{0.0F};

    PointXYdZ() = default;
    PointXYdZ(double x, double y) : x(x), y(y)
    {
    }
    PointXYdZ(double x, double y, float z_min, float z_max) : x(x), y(y), z_min(z_min), z_max(z_max)
    {
    }
};

struct PolygonizationConfiguration final
{
    bool enable_concave_hulling{true};
    bool convex_hull_small_clusters{true};
    std::uint32_t max_polygon_points{300U};
    std::uint32_t small_cluster_point_count{20U};
    double concave_hull_chi_parameter{0.2};
};

class Polygonizer final
{
  public:
    Polygonizer() = default;
    ~Polygonizer() = default;

    void update_configuration(const PolygonizationConfiguration &configuration);

    void reserve_memory(std::uint32_t max_points_per_cluster = 200'000U);

    template <typename PointT>
    void polygonize(const pcl::PointCloud<PointT> &cloud, const std::vector<ClusteringLabel> &cluster_labels,
                    containers::Vector<containers::Vector<PointXYdZ>> &polygons);

  private:
    PolygonizationConfiguration configuration_;
    containers::Vector<PointXYdZ> polygon_;
    std::vector<double> coordinates_xy_;
};

extern template void Polygonizer::polygonize(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                             const std::vector<ClusteringLabel> &cluster_labels,
                                             containers::Vector<containers::Vector<PointXYdZ>> &polygons);

extern template void Polygonizer::polygonize(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                             const std::vector<ClusteringLabel> &cluster_labels,
                                             containers::Vector<containers::Vector<PointXYdZ>> &polygons);

extern template void Polygonizer::polygonize(const pcl::PointCloud<pcl::PointXYZL> &cloud,
                                             const std::vector<ClusteringLabel> &cluster_labels,
                                             containers::Vector<containers::Vector<PointXYdZ>> &polygons);

extern template void Polygonizer::polygonize(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                             const std::vector<ClusteringLabel> &cluster_labels,
                                             containers::Vector<containers::Vector<PointXYdZ>> &polygons);

extern template void Polygonizer::polygonize(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud,
                                             const std::vector<ClusteringLabel> &cluster_labels,
                                             containers::Vector<containers::Vector<PointXYdZ>> &polygons);
} // namespace lidar_processing

#endif // LIDAR_PROCESSING__POLYGONIZATION_HPP
