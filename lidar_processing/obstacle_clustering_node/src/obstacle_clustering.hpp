#pragma once
#ifndef OBSTACLE_CLUSTERING_HPP_
#define OBSTACLE_CLUSTERING_HPP_

// new point types
#include "point_types.hpp"

// std
#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <numeric>
#include <random>
#include <vector>

// Eigen
#include <eigen3/Eigen/Dense>

// PCL
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace lidar_processing
{
class ObstacleClustering : public pcl::PCLBase<pcl::PointXYZ>
{
  public:
    ObstacleClustering(double neighbour_radius_threshold = 0.2, unsigned int maximum_neighbour_points = 1000U,
                       unsigned int minimum_neighbour_points_threshold = 4)
        : neighbour_radius_threshold_(neighbour_radius_threshold), maximum_neighbour_points_(maximum_neighbour_points),
          minimum_neighbour_points_threshold_(minimum_neighbour_points_threshold){};

    ~ObstacleClustering() = default;

    void clusterObstacles(const pcl::PointCloud<pcl::PointXYZRGBI> &cloud,
                          pcl::PointCloud<pcl::PointXYZRGBL> &clustered_cloud);

  private:
    double neighbour_radius_threshold_;
    unsigned int maximum_neighbour_points_;
    unsigned int minimum_neighbour_points_threshold_;
};
} // namespace lidar_processing

#endif // OBSTACLE_CLUSTERING_HPP_