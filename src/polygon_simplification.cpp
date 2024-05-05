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

#include "polygon_simplification.hpp"

// Internal
#include "concave_hull.hpp"

#define DEBUG_POLYGONIZATION 0

namespace lidar_processing
{
void findOrderedConvexOutlines(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &clustered_obstacle_cloud,
                               std::vector<std::vector<geom::Point<float>>> &convex_hulls)
{
    using PointType = geom::Point<float>;

    // Clear old message and reserve space
    convex_hulls.clear();
    convex_hulls.reserve(clustered_obstacle_cloud.size());

    // Copy data into suitable format
    for (const auto &cluster : clustered_obstacle_cloud)
    {
        // Copy points from current cluster
        std::vector<PointType> cluster_points;
        cluster_points.reserve(cluster.size());
        for (const auto &point : cluster.points)
        {
            cluster_points.push_back(PointType(point.x, point.y));
        }

        // Construct convex hull
        std::vector<std::int32_t> hull_indices;
        if (cluster_points.size() > 1000)
        {
            hull_indices = std::move(geom::constructConvexHull(cluster_points, geom::ConvexHullAlgorithm::CHAN,
                                                               geom::Orientation::COUNTERCLOCKWISE));
        }
        else
        {
            hull_indices = std::move(geom::constructConvexHull(
                cluster_points, geom::ConvexHullAlgorithm::ANDREW_MONOTONE_CHAIN, geom::Orientation::COUNTERCLOCKWISE));
        }

        std::vector<PointType> hull_points;
        for (const auto index : hull_indices)
        {
            hull_points.push_back(cluster_points[index]);
        }

#if DEBUG_POLYGONIZATION
        std::cout << "Convex hull contains " << hull_points.size() << " points." << std::endl;
#endif

        if (!hull_points.empty())
        {
            convex_hulls.push_back(std::move(hull_points));
        }
    }
}

void findOrderedConcaveOutlines(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &clustered_obstacle_cloud,
                                std::vector<std::vector<geom::Point<float>>> &concave_hulls)
{
    using PointType = geom::Point<float>;

    // Clear old message and reserve space
    concave_hulls.clear();
    concave_hulls.reserve(clustered_obstacle_cloud.size());

    // Copy data into suitable format
    for (const auto &cluster : clustered_obstacle_cloud)
    {
        // Point cache
        std::vector<PointType> hull_points;

        // Apply convex hulling instead
        if (cluster.size() < 20)
        {
            std::vector<PointType> cluster_points;
            cluster_points.reserve(cluster.size());
            for (const auto &point : cluster.points)
            {
                cluster_points.push_back(PointType{point.x, point.y});
            }

            const auto hull_indices = geom::constructConvexHull(
                cluster_points, geom::ConvexHullAlgorithm::ANDREW_MONOTONE_CHAIN, geom::Orientation::COUNTERCLOCKWISE);

            hull_points.reserve(hull_indices.size());
            for (const auto index : hull_indices)
            {
                hull_points.push_back(cluster_points[index]);
            }
        }
        // Apply concave hulling
        else
        {
            // Copy points from current cluster
            std::vector<float> coordinates;
            coordinates.reserve(cluster.size() * 2);
            for (const auto &point : cluster.points)
            {
                coordinates.push_back(point.x);
                coordinates.push_back(point.y);
            }

            // Construct concave hull
            geometry::ConcaveHull hull(coordinates, 0.2);
            const auto hull_indices = hull.getHullIndices();

            hull_points.reserve(hull_indices.size());
            for (const auto index : hull_indices)
            {
                const auto &cluster_point = cluster[index];
                hull_points.push_back(PointType{cluster_point.x, cluster_point.y});
            }
        }

#if DEBUG_POLYGONIZATION
        std::cout << "Concave hull contains " << hull_points.size() << " points." << std::endl;
#endif

        if (!hull_points.empty())
        {
            concave_hulls.push_back(std::move(hull_points));
        }
    }
}
} // namespace lidar_processing
