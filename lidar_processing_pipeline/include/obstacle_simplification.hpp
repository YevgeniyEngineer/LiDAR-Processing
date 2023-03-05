#ifndef OBSTACLE_SIMPLIFICATION
#define OBSTACLE_SIMPLIFICATION

// Internal
#include "geometric_operations.hpp" // constructGrahamAndrewConvexHull
#include "internal_types.hpp"

// STL
#include <vector> // std::vector

// PCL
#include <pcl/point_cloud.h> // pcl::PointCloud
#include <pcl/point_types.h> // pcl::PointXYZ

namespace lidar_processing
{
inline void findOrderedConvexOutline(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &clustered_obstacle_cloud,
                                     std::vector<std::vector<PointXY>> &convex_hulls)
{
    convex_hulls.clear();
    convex_hulls.reserve(clustered_obstacle_cloud.size());

    for (int cluster_no = 0; cluster_no < clustered_obstacle_cloud.size(); ++cluster_no)
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
        if (cluster_points.size() > 200)
        {
            constructChanConvexHull(std::move(cluster_points), hull_points);
        }
        else
        {
            constructGrahamAndrewConvexHull(std::move(cluster_points), hull_points);
        }

        convex_hulls.emplace_back(std::move(hull_points));
    }
}
} // namespace lidar_processing

#endif // OBSTACLE_SIMPLIFICATION