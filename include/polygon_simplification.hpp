#ifndef POLYGON_SIMPLIFICATION_HPP
#define POLYGON_SIMPLIFICATION_HPP

// Internal
#include "convex_hull.hpp"

// STL
#include <cstdint>  // std::size_t
#include <iostream> // std::cout
#include <vector>   // std::vector

// PCL
#include <pcl/point_cloud.h> // pcl::PointCloud
#include <pcl/point_types.h> // pcl::PointXYZ

namespace lidar_processing
{
void findOrderedConvexOutline(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &clustered_obstacle_cloud,
                              std::vector<std::vector<geom::Point<float>>> &convex_hulls);
} // namespace lidar_processing

#endif // POLYGON_SIMPLIFICATION_HPP