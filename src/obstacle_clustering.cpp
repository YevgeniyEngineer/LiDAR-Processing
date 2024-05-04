#include "obstacle_clustering.hpp"

namespace lidar_processing
{
ObstacleClusterer::ObstacleClusterer(float neighbour_radius_threshold, float cluster_quality,
                                     std::uint32_t min_cluster_size, std::uint32_t max_cluster_size,
                                     ClusteringAlgorithm clustering_algorithm)
    : neighbour_radius_threshold_(neighbour_radius_threshold), cluster_quality_(cluster_quality),
      min_cluster_size_(min_cluster_size), max_cluster_size_(max_cluster_size),
      clustering_algorithm_(clustering_algorithm),
      dbscan_clusterer_(std::make_unique<clustering::DBSCANClustering<float, 3>>(neighbour_radius_threshold_,
                                                                                 min_cluster_size_, max_cluster_size_))
{
}

ObstacleClusterer::~ObstacleClusterer()
{
}

void ObstacleClusterer::clusterObstacles(const pcl::PointCloud<pcl::PointXYZRGBL> &obstacle_cloud,
                                         std::vector<pcl::PointCloud<pcl::PointXYZ>> &clustered_cloud)
{
    clustered_cloud.clear();

    if (obstacle_cloud.empty())
    {
        std::cerr << "No obstacles to cluster" << std::endl;
        return;
    }

    using CoordinateType = decltype(obstacle_cloud.points[0].x);

    if (clustering_algorithm_ == ClusteringAlgorithm::DBSCAN)
    {
        using DBSCANPoint = clustering::DBSCANPoint<CoordinateType, 3>;
        using DBSCANPointCloud = clustering::DBSCANPointCloud<CoordinateType, 3>;
        using DBSCANClustering = clustering::DBSCANClustering<CoordinateType, 3>;

        auto point_cloud = std::make_unique<DBSCANPointCloud>();
        for (const auto &point : obstacle_cloud.points)
        {
            point_cloud->points.push_back(DBSCANPoint{point.x, point.y, point.z});
        }

        dbscan_clusterer_->reserve(point_cloud->points.size() * 2);

        dbscan_clusterer_->rebuildKDTree(*point_cloud);

        dbscan_clusterer_->formClusters();

        const auto clusters = dbscan_clusterer_->getClusterIndices();

        if (!clusters.empty())
        {
            clustered_cloud.reserve(clusters.size());

            for (const auto &[cluster_label, cluster_indices] : clusters)
            {
                if (cluster_label == clustering::labels::NOISE)
                {
                    continue;
                }

                auto cluster_points = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
                cluster_points->reserve(cluster_indices.size());

                for (const auto cluster_index : cluster_indices)
                {
                    const auto &point = point_cloud->points[cluster_index];
                    cluster_points->push_back(pcl::PointXYZ{point[0], point[1], point[2]});
                }

                cluster_points->height = 1;
                cluster_points->width = cluster_points->points.size();

                clustered_cloud.push_back(std::move(*cluster_points));
                cluster_points.reset();
            }
        }
    }
    else if (clustering_algorithm_ == ClusteringAlgorithm::FAST_EUCLIDEAN_CLUSTERING)
    {
        using FECPoint = clustering::FECPoint<CoordinateType, 3>;
        using FECPointCloud = clustering::FECPointCloud<CoordinateType, 3>;
        using FECClustering = clustering::FECClustering<CoordinateType, 3>;

        auto point_cloud = std::make_unique<FECPointCloud>();
        point_cloud->reserve(obstacle_cloud.size());
        for (const auto &point : obstacle_cloud.points)
        {
            point_cloud->push_back(FECPoint{point.x, point.y, point.z});
        }

        auto fec_clustering = std::make_unique<FECClustering>(*point_cloud, neighbour_radius_threshold_,
                                                              min_cluster_size_, max_cluster_size_, cluster_quality_);

        fec_clustering->formClusters();

        const auto clusters = fec_clustering->getClusterIndices();

        if (!clusters.empty())
        {
            clustered_cloud.reserve(clusters.size());
            for (const auto &[cluster_label, cluster_indices] : clusters)
            {
                auto cluster_points = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
                cluster_points->reserve(cluster_indices.size());

                for (const auto &cluster_index : cluster_indices)
                {
                    const auto &point = (*point_cloud)[cluster_index];
                    cluster_points->push_back(pcl::PointXYZ{point[0], point[1], point[2]});
                }

                cluster_points->height = 1;
                cluster_points->width = cluster_points->points.size();

                clustered_cloud.push_back(std::move(*cluster_points));
                cluster_points.reset();
            }
        }
    }
    else if (clustering_algorithm_ == ClusteringAlgorithm::ADAPTIVE_EUCLIDEAN_CLUSTERING)
    {
        const float distance_neighbour_threshold_increment = neighbour_radius_threshold_ / 2.0f;

        formAdaptiveEuclideanCluster(obstacle_cloud, clustered_cloud, distance_neighbour_threshold_increment,
                                     min_cluster_size_, max_cluster_size_, cluster_quality_);
    }
    else if (clustering_algorithm_ == ClusteringAlgorithm::ADAPTIVE_DBSCAN)
    {
        using CoordinateType = float;
        using AdaptiveDBSCANType = clustering::AdaptiveDBSCANClustering<CoordinateType>;
        using AdaptiveDBSCANPointCloudType = clustering::AdaptiveDBSCANPointCloud<CoordinateType>;
        using PointCloudType = AdaptiveDBSCANPointCloudType::PointCloud;

        // Transform to coordinates
        std::vector<CoordinateType> coordinates;
        const std::uint32_t number_of_coordinates = 3 * obstacle_cloud.points.size();
        coordinates.reserve(number_of_coordinates);
        for (const auto &point : obstacle_cloud.points)
        {
            coordinates.push_back(point.x);
            coordinates.push_back(point.y);
            coordinates.push_back(point.z);
        }

        // Construct DBSCAN class
        AdaptiveDBSCANType dbscan{1.0F, 0.4F, 2.5F, 4U};

        // Create point cloud adapter
        const AdaptiveDBSCANPointCloudType point_cloud{coordinates};

        // Create KDTree index
        dbscan.createIndex(point_cloud);

        // Cluster points
        dbscan.formClusters();

        // Get cluster labels
        const auto &labels = dbscan.labels();

        // Find maximum label
        const auto max_label = *std::max_element(labels.cbegin(), labels.cend());

        // Iterate through labels
        for (std::int32_t label = 0; label <= max_label; ++label)
        {
            // Iterate through points
            pcl::PointCloud<pcl::PointXYZ> cluster_cloud;
            cluster_cloud.reserve(obstacle_cloud.points.size());
            for (std::uint32_t point_number = 0U; point_number < obstacle_cloud.points.size(); ++point_number)
            {
                if (labels[point_number] == label)
                {
                    const auto &point = obstacle_cloud.points[point_number];
                    cluster_cloud.points.emplace_back(point.x, point.y, point.z);
                }
            }

            // Move points
            clustered_cloud.push_back(std::move(cluster_cloud));
        }
    }
}

} // namespace lidar_processing
