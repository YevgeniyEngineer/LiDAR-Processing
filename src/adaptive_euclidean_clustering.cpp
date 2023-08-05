#include "adaptive_euclidean_clustering.hpp"

#include "fec_clustering.hpp"

#include <array>
#include <cmath>

namespace lidar_processing
{
using namespace clustering;

using DataType = float;
static constexpr std::size_t NUMBER_OF_DIMENSIONS = 3U;

using FECPointType = FECPoint<DataType, NUMBER_OF_DIMENSIONS>;
using FECPointCloudType = FECPointCloud<DataType, NUMBER_OF_DIMENSIONS>;
using FECClusteringType = FECClustering<DataType, NUMBER_OF_DIMENSIONS>;

// Kitti uses HDL-64E Velodyne
static constexpr std::size_t number_of_regions = 5U;
static constexpr float regions[number_of_regions] = {5.0f, 12.0f, 20.0f, 40.0f,
                                                     50.0f};

static std::vector<FECPointCloudType> divideCloudIntoCircularRegions(
    const pcl::PointCloud<pcl::PointXYZRGBL>& obstacle_cloud,
    float incremental_cluster_tolerance_m)
{
    // Pre-compute squared distances for each region
    std::array<float, number_of_regions> region_squared_distances;

    float distance_offset = 0.0f;
    for (std::size_t i = 0U; i < number_of_regions; ++i)
    {
        distance_offset += regions[i];
        region_squared_distances[i] = distance_offset * distance_offset;
    }

    // Output cloud
    std::vector<FECPointCloudType> region_partitioned_cloud(number_of_regions);

    // Partition points into regions
    std::size_t point_index = 0U;
    for (const auto& point : obstacle_cloud.points)
    {
        // Calculate squared distance from origin
        // Assume origin is located at (0, 0, 0)
        const float distance_squared =
            point.x * point.x + point.y * point.y + point.z * point.z;

        float previous_region_square = 0.0f;

        bool point_added =
            false; // flag to check if the point was added to a region

        for (std::size_t region_number = 0U; region_number < number_of_regions;
             ++region_number)
        {
            if (distance_squared > previous_region_square &&
                distance_squared <= region_squared_distances[region_number])
            {
                region_partitioned_cloud[region_number].push_back(
                    FECPointType{point.x, point.y, point.z});

                point_added = true; // point was added

                break; // exit once we find the region
            }

            if (region_number > 0U)
            {
                previous_region_square =
                    region_squared_distances[region_number - 1U];
            }
        }

        // If point was not added to any region, add it to the last region
        if (!point_added)
        {
            region_partitioned_cloud[number_of_regions - 1U].push_back(
                FECPointType{point.x, point.y, point.z});
        }

        ++point_index;
    }

    return region_partitioned_cloud;
}

void formAdaptiveEuclideanCluster(
    const pcl::PointCloud<pcl::PointXYZRGBL>& obstacle_cloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>>& clustered_cloud,
    float incremental_cluster_tolerance_m, std::uint32_t min_cluster_size,
    std::uint32_t max_cluster_size, float cluster_quality)
{
    // Clear cache
    clustered_cloud.clear();

    // Partition cloud
    const auto& region_partitioned_cloud = divideCloudIntoCircularRegions(
        obstacle_cloud, incremental_cluster_tolerance_m);

    // Iterate through each region
    float cluster_tolerance = 0.0f;
    for (std::size_t region_number = 0U; region_number < number_of_regions;
         ++region_number)
    {
        cluster_tolerance += incremental_cluster_tolerance_m;

        const auto& region_cloud = region_partitioned_cloud[region_number];
        if (region_cloud.size() > min_cluster_size)
        {
            // Apply Euclidean clustering on the local region
            FECClusteringType fec_clustering{region_cloud, cluster_tolerance,
                                             min_cluster_size, max_cluster_size,
                                             cluster_quality};

            fec_clustering.formClusters();

            const auto clusters = fec_clustering.getClusterIndices();

            if (!clusters.empty())
            {
                clustered_cloud.reserve(clusters.size());
                for (const auto& [cluster_label, cluster_indices] : clusters)
                {
                    auto cluster_points =
                        std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
                    cluster_points->reserve(cluster_indices.size());

                    for (const auto& cluster_index : cluster_indices)
                    {
                        const auto& point = region_cloud[cluster_index];
                        cluster_points->push_back(
                            pcl::PointXYZ{point[0], point[1], point[2]});
                    }

                    cluster_points->height = 1;
                    cluster_points->width = cluster_points->points.size();

                    clustered_cloud.push_back(std::move(*cluster_points));
                    cluster_points.reset();
                }
            }
        }
        // Not enough points to form a cluster
        else
        {
        }
    }
}
} // namespace lidar_processing