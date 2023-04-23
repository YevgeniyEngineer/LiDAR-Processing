#ifndef OBSTACLE_SIMPLIFICATION
#define OBSTACLE_SIMPLIFICATION

// Internal
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
                                     std::vector<std::vector<PointXY>> &convex_hulls)
{
    convex_hulls.clear();
    convex_hulls.reserve(clustered_obstacle_cloud.size());

    for (std::size_t cluster_no = 0; cluster_no < clustered_obstacle_cloud.size(); ++cluster_no)
    {
        const auto &cluster = clustered_obstacle_cloud[cluster_no];
        std::vector<PointXY> cluster_points;
        cluster_points.reserve(cluster.points.size());
        for (const auto &point : cluster.points)
        {
            PointXY point_cache;
            point_cache.x = point.x;
            point_cache.y = point.y;
            cluster_points.emplace_back(std::move(point_cache));
        }

        std::vector<PointXY> hull_points;
        if (cluster_points.size() > 1000)
        {
#if DEBUG_CONVEX_POLYGONIZATION
            std::cout << "Constructing convex hull using Chan's algorithm from " << cluster_points.size() << " points"
                      << std::endl;
#endif
            constructChanConvexHull(std::move(cluster_points), hull_points);
        }
        else
        {
#if DEBUG_CONVEX_POLYGONIZATION
            std::cout << "Constructing convex hull using Andrew's algorithm from " << cluster_points.size() << " points"
                      << std::endl;
#endif
            constructGrahamAndrewConvexHull(std::move(cluster_points), hull_points);
        }

        // Close the hull
        if (!hull_points.empty())
        {
#if DEBUG_CONVEX_POLYGONIZATION
            std::cout << "Convex hull contains " << hull_points.size() << " points." << std::endl;
#endif
            hull_points.emplace_back(hull_points[0]);
            convex_hulls.emplace_back(std::move(hull_points));
        }
    }
}
} // namespace lidar_processing

#endif // OBSTACLE_SIMPLIFICATION