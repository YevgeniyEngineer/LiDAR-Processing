#ifndef OBSTACLE_SIMPLIFICATION
#define OBSTACLE_SIMPLIFICATION

// Internal
#include "convex_hull.hpp"

// STL
#include <vector>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
inline void findOrderedConvexOutline(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &clustered_obstacle_cloud,
                                     std::vector<std::vector<point_t>> &convex_hulls)
{
    convex_hulls.clear();
    convex_hulls.reserve(clustered_obstacle_cloud.size());

    for (int cluster_no = 0; cluster_no < clustered_obstacle_cloud.size(); ++cluster_no)
    {
        const auto &cluster = clustered_obstacle_cloud[cluster_no];
        std::size_t cluster_point_no = 0;
        std::vector<point_t> cluster_points;
        cluster_points.reserve(cluster.points.size());
        for (const auto &point : cluster.points)
        {
            point_t point_cache;
            point_cache.x = point.x;
            point_cache.y = point.y;
            point_cache.index = cluster_point_no;
            cluster_points.emplace_back(std::move(point_cache));
            ++cluster_point_no;
        }

        ConvexHull convex_hull_generator(cluster_points);
        std::vector<point_t> hull_points;
        convex_hull_generator.getConvexHull(hull_points);
        convex_hulls.emplace_back(std::move(hull_points));
    }
}
} // namespace lidar_processing

#endif // OBSTACLE_SIMPLIFICATION