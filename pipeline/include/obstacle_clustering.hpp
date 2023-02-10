#ifndef OBSTACLE_CLUSTERING_HPP_
#define OBSTACLE_CLUSTERING_HPP_

// Required libraries
#include "euclidean_clustering.hpp"

// std
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
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
class ObstacleClusterer : public pcl::PCLBase<pcl::PointXYZ>
{
  public:
    ObstacleClusterer(double neighbour_radius_threshold = 0.8, double cluster_quality = 0.1,
                      unsigned int min_cluster_size = 10, unsigned int max_neighbour_points = 1000000,
                      unsigned int min_neighbour_points_threshold = 4)
        : neighbour_radius_threshold_(neighbour_radius_threshold), cluster_quality_(cluster_quality),
          min_cluster_size_(min_cluster_size), max_neighbour_points_(max_neighbour_points),
          min_neighbour_points_threshold_(min_neighbour_points_threshold){};

    ~ObstacleClusterer() = default;

    template <typename PointT>
    void clusterObstacles(const typename pcl::PointCloud<PointT> &cloud,
                          pcl::PointCloud<pcl::PointXYZRGBL> &clustered_cloud);

  private:
    double neighbour_radius_threshold_;
    double cluster_quality_;
    unsigned int min_cluster_size_;
    unsigned int max_neighbour_points_;
    unsigned int min_neighbour_points_threshold_;
};

// Perform clustering on point cloud, thus forming cluster groups
// These cluster groups can represent individual object in the scene
template <typename PointT>
void ObstacleClusterer::clusterObstacles(const typename pcl::PointCloud<PointT> &cloud,
                                         pcl::PointCloud<pcl::PointXYZRGBL> &clustered_cloud)
{
    if (cloud.empty())
    {
        return;
    }

    const auto &number_of_points = cloud.size();

    // Copy cloud into PointXYZ for faster processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (const auto point : cloud.points)
    {
        pcl::PointXYZ point_cache{point.x, point.y, point.z};
        cloud_xyz->points.emplace_back(std::move(point_cache));
    }
    cloud_xyz->height = 1;
    cloud_xyz->width = cloud_xyz->points.size();

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    std::vector<pcl::PointIndices> cluster_indices;

    FastEuclideanClustering<pcl::PointXYZ> fast_euclidean_clustering;
    fast_euclidean_clustering.setInputCloud(cloud_xyz);
    fast_euclidean_clustering.setSearchMethod(kdtree);
    fast_euclidean_clustering.setClusterTolerance(neighbour_radius_threshold_);
    fast_euclidean_clustering.setQuality(cluster_quality_);
    fast_euclidean_clustering.setMinClusterSize(min_cluster_size_);
    fast_euclidean_clustering.segment(cluster_indices);

    // std::cout << "#clusters: " << cluster_indices.size() << std::endl;
    clustered_cloud.clear();
    clustered_cloud.reserve(cloud_xyz->size());

    std::uint32_t cluster_label = 0;
    for (const auto &cluster : cluster_indices)
    {
        // generate random color for this cluster
        const uint8_t r_color = static_cast<uint8_t>(static_cast<double>(std::rand()) / RAND_MAX * 255);
        const uint8_t g_color = static_cast<uint8_t>(static_cast<double>(std::rand()) / RAND_MAX * 255);
        const uint8_t b_color = static_cast<uint8_t>(static_cast<double>(std::rand()) / RAND_MAX * 255);

        for (const auto &index : cluster.indices)
        {
            const auto &point_xyz = cloud_xyz->points[index];

            clustered_cloud.emplace_back(point_xyz.x, point_xyz.y, point_xyz.z, r_color, g_color, b_color,
                                         cluster_label);
        }
        ++cluster_label;
    }
}

} // namespace lidar_processing

#endif // OBSTACLE_CLUSTERING_HPP_