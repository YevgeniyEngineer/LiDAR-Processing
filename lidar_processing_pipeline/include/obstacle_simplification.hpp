#ifndef OBSTACLE_SIMPLIFICATION
#define OBSTACLE_SIMPLIFICATION

// Internal
#include "convex_hull.hpp"
#include "geometric_operations.hpp" // constructGrahamAndrewConvexHull
#include "internal_types.hpp"

// STL
#include <cstdint>  // std::size_t
#include <iostream> // std::cout
#include <vector>   // std::vector

// PCL
#include <pcl/point_cloud.h> // pcl::PointCloud
#include <pcl/point_types.h> // pcl::PointXYZ

#define DEBUG_CONVEX_POLYGONIZATION 0

namespace lidar_processing
{
inline void findOrderedConvexOutline(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &clustered_obstacle_cloud,
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

#if DEBUG_CONVEX_POLYGONIZATION
        std::cout << "Convex hull contains " << hull_points.size() << " points." << std::endl;
#endif

        convex_hulls.push_back(std::move(hull_points));
    }
}
} // namespace lidar_processing

#endif // OBSTACLE_SIMPLIFICATION