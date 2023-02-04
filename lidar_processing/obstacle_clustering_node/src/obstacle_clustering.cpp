#include "obstacle_clustering.hpp"
#include "fast_euclidean_clustering.hpp"

namespace lidar_processing
{
// Perform clustering on the input point cloud, and place clustered clouds into pcl::PointXYZRGBL
// Based on DBScan
void ObstacleClustering::clusterObstacles(const pcl::PointCloud<pcl::PointXYZRGBI> &cloud,
                                          pcl::PointCloud<pcl::PointXYZRGBL> &clustered_cloud)
{
    size_t number_of_points = cloud.size();

    // copy cloud into this cloud, that will hold intensities and labels
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::vector<float> intensities;
    std::vector<std::int32_t> labels(number_of_points, 0); // initializes all elements to 0
    input_cloud->points.reserve(number_of_points);

    for (const pcl::PointXYZRGBI &point : cloud.points)
    {
        pcl::PointXYZ point_cache;
        point_cache.x = point.x;
        point_cache.y = point.y;
        point_cache.z = point.z;

        intensities.emplace_back(point.intensity);
        input_cloud->points.emplace_back(std::move(point_cache));
    }
    input_cloud->height = 1;
    input_cloud->width = input_cloud->points.size();

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    std::vector<pcl::PointIndices> cluster_indices;

    double cluster_tolerance = 0.8;
    double quality = 0.1;

    FastEuclideanClustering<pcl::PointXYZ> fast_euclidean_clustering;
    fast_euclidean_clustering.setInputCloud(input_cloud);
    fast_euclidean_clustering.setSearchMethod(kdtree);
    fast_euclidean_clustering.setClusterTolerance(cluster_tolerance);
    fast_euclidean_clustering.setQuality(quality);
    fast_euclidean_clustering.setMinClusterSize(10);
    fast_euclidean_clustering.segment(cluster_indices);

    std::cout << "#clusters: " << cluster_indices.size() << std::endl;

    // for (const auto &cluster_index : cluster_indices)
    // {
    //     std::cout << "Number of elements within a cluster: " << cluster_index.indices.size() << std::endl;
    // }

    // form clustered_cloud
    // each cluster will have a random RGB value and label
    for (std::uint32_t cluster_no = 0; cluster_no < static_cast<std::uint32_t>(cluster_indices.size()); ++cluster_no)
    {
        // indices pointing to the original point cloud points
        const auto &cluster_index_array = cluster_indices[cluster_no].indices;

        // generate random color
        uint8_t r_color = static_cast<uint8_t>(static_cast<double>(std::rand()) / RAND_MAX * 255);
        uint8_t g_color = static_cast<uint8_t>(static_cast<double>(std::rand()) / RAND_MAX * 255);
        uint8_t b_color = static_cast<uint8_t>(static_cast<double>(std::rand()) / RAND_MAX * 255);

        for (const auto &point_index : cluster_index_array)
        {
            const pcl::PointXYZRGBI &point = cloud.points[point_index];
            pcl::PointXYZRGBL point_cache;
            point_cache.x = point.x;
            point_cache.y = point.y;
            point_cache.z = point.z;
            point_cache.r = r_color;
            point_cache.g = g_color;
            point_cache.b = b_color;
            point_cache.label = cluster_no;
            clustered_cloud.points.emplace_back(std::move(point_cache));
        }
    }
    clustered_cloud.height = 1;
    clustered_cloud.width = clustered_cloud.points.size();
}
} // namespace lidar_processing