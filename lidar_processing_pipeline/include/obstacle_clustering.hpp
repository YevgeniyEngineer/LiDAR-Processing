#ifndef OBSTACLE_CLUSTERING_HPP_
#define OBSTACLE_CLUSTERING_HPP_

// Required libraries
#include "dbscan.hpp"
#include "fast_euclidean_clustering.hpp"

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

// PCL
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#define DEBUG_CLUSTERING 0

namespace lidar_processing
{
enum class ClusteringAlgorithm : std::uint8_t
{
    DBSCAN,
    FAST_EUCLIDEAN_CLUSTERING
};

class ObstacleClusterer : public pcl::PCLBase<pcl::PointXYZ>
{
  public:
    ObstacleClusterer(double neighbour_radius_threshold = 0.8, double cluster_quality = 0.5,
                      unsigned int min_cluster_size = 10, unsigned int max_neighbour_points = 100000,
                      unsigned int min_neighbour_points_threshold = 4,
                      ClusteringAlgorithm clustering_algorithm = ClusteringAlgorithm::DBSCAN)
        : neighbour_radius_threshold_(neighbour_radius_threshold), cluster_quality_(cluster_quality),
          min_cluster_size_(min_cluster_size), max_neighbour_points_(max_neighbour_points),
          min_neighbour_points_threshold_(min_neighbour_points_threshold),
          clustering_algorithm_(clustering_algorithm){};

    ~ObstacleClusterer() = default;

    void setNeighbourRadiusThreshold(double neighbour_radius_threshold)
    {
        neighbour_radius_threshold_ = neighbour_radius_threshold;
    }

    void setMinClusterSize(unsigned int min_cluster_size)
    {
        min_cluster_size_ = min_cluster_size;
    }

    void setClusteringAlgorithm(ClusteringAlgorithm clustering_algorithm)
    {
        clustering_algorithm_ = clustering_algorithm;
    }

    template <typename PointT>
    void clusterObstacles(const typename pcl::PointCloud<PointT> &cloud,
                          std::vector<pcl::PointCloud<pcl::PointXYZ>> &clustered_cloud);

  private:
    double neighbour_radius_threshold_;
    double cluster_quality_;
    unsigned int min_cluster_size_;
    unsigned int max_neighbour_points_;
    unsigned int min_neighbour_points_threshold_;
    ClusteringAlgorithm clustering_algorithm_;
};

// Perform clustering on point cloud, thus forming cluster groups
// These cluster groups can represent individual object in the scene
template <typename PointT>
void ObstacleClusterer::clusterObstacles(const pcl::PointCloud<PointT> &cloud,
                                         std::vector<pcl::PointCloud<pcl::PointXYZ>> &clustered_cloud)
{
    if (cloud.empty())
    {
        return;
    }

    if (clustering_algorithm_ == ClusteringAlgorithm::DBSCAN)
    {
        using CoordinateType = decltype(cloud.points[0].x);

        // Prepare points for clustering
        auto point_cloud = std::make_unique<clustering::PointCloud<CoordinateType, 3>>();
        for (const auto &point : cloud.points)
        {
            clustering::Point<CoordinateType, 3> point_cache{point.x, point.y, point.z};
            point_cloud->points.emplace_back(std::move(point_cache));
        }

#if DEBUG_CLUSTERING
        std::cout << "Number of points to be clustered: " << point_cloud->points.size() << std::endl;
#endif

        // Create DBScan object and start clustering
        auto dbscan = std::make_unique<clustering::dbscan::DBSCAN<CoordinateType, 3>>(neighbour_radius_threshold_,
                                                                                      min_cluster_size_, *point_cloud);

#if DEBUG_CLUSTERING
        std::cout << "Created DBSCAN Object" << std::endl;
#endif

        dbscan->formClusters();
        const auto clusters = dbscan->getClusterIndices();

#if DEBUG_CLUSTERING
        std::cout << "Created " << clusters.size() << " clusters." << std::endl;
#endif

        // Copy points to output cloud
        clustered_cloud.clear();
        clustered_cloud.reserve(clusters.size());
        for (const auto &cluster_indices : clusters)
        {
            // Skip noise points
            if (cluster_indices.first == clustering::dbscan::labels::NOISE)
            {
                continue;
            }
            else if (cluster_indices.first == clustering::dbscan::labels::UNDEFINED)
            {
                std::cerr << "DBSCAN added UNDEFINED cluster\n";
                continue;
            }

            // Add non-noise points
            auto cluster_points = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
            cluster_points->reserve(cluster_indices.second.size());
            for (const auto &cluster_index : cluster_indices.second)
            {
                // Get the point
                const auto &point = point_cloud->points[cluster_index];
                pcl::PointXYZ point_cache{point[0], point[1], point[2]};
                cluster_points->emplace_back(std::move(point_cache));
            }
            cluster_points->height = 1;
            cluster_points->width = cluster_points->points.size();
            clustered_cloud.emplace_back(std::move(*cluster_points));
            cluster_points.reset();
        }

#if DEBUG_CLUSTERING
        std::cout << "Copied " << clustered_cloud.size() << " clusters." << std::endl;
#endif
    }
    else if (clustering_algorithm_ == ClusteringAlgorithm::FAST_EUCLIDEAN_CLUSTERING)
    {
        // Prepare points for clustering
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        input_cloud->points.reserve(cloud.size());
        for (const auto &point : cloud.points)
        {
            pcl::PointXYZ point_cache{point.x, point.y, point.z};
            input_cloud->points.emplace_back(std::move(point_cache));
        }
        input_cloud->height = 1;
        input_cloud->width = input_cloud->points.size();

        // Perform clustering and store cluster indices
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

        std::vector<pcl::PointIndices> cluster_indices;
        FastEuclideanClustering<pcl::PointXYZ> fast_euclidean_clustering;
        fast_euclidean_clustering.setInputCloud(input_cloud);
        fast_euclidean_clustering.setSearchMethod(kdtree);
        fast_euclidean_clustering.setClusterTolerance(neighbour_radius_threshold_);
        fast_euclidean_clustering.setQuality(cluster_quality_);
        fast_euclidean_clustering.setMinClusterSize(min_cluster_size_);
        fast_euclidean_clustering.segment(cluster_indices);

        // Copy clustered points
        clustered_cloud.clear();
        clustered_cloud.reserve(cluster_indices.size());
        for (const auto &cluster_index_list : cluster_indices)
        {
            auto cluster_points = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
            cluster_points->reserve(cluster_index_list.indices.size());
            for (const auto &index : cluster_index_list.indices)
            {
                const pcl::PointXYZ &point_xyz = input_cloud->points[index];
                pcl::PointXYZ point_cache{point_xyz.x, point_xyz.y, point_xyz.z};
                cluster_points->emplace_back(std::move(point_cache));
            }
            cluster_points->height = 1;
            cluster_points->width = cluster_points->points.size();
            clustered_cloud.emplace_back(std::move(*cluster_points));
            cluster_points.reset();
        }
    }
}

} // namespace lidar_processing

#endif // OBSTACLE_CLUSTERING_HPP_