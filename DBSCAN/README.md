# DBSCAN

Very fast sequential C++ implementation of the DBSCAN algorithm based on Nanoflann v1.4.3 library for nearest neighbor search.

# Requirements

`sudo apt install libnanoflann-dev`

# To Update Submodule

`git submodule update --init --recursive`

# Benchmarks for 3D DBSCAN:

CPU: 12th Gen Intel® Core™ i7-12700H × 20 | 16.0 GiB RAM

| Number of Points | Average Clustering Time (s) |
| ---------------- | --------------------------- |
| 1,000            | 92.330e-06                  |
| 10,000           | 138.30e-06                  |
| 50,000           | 10.565e-03                  |
| 100,000          | 22.980e-03                  |
| 1,000,000        | 799.613e-03                 |

# Example Usage:

```cpp
#include "dbscan.hpp"

int main() 
{
    // DBSCAN configuration parameters
    constexpr double SEARCH_RADIUS = 1.0;
    constexpr int NUMBER_OF_POINTS_TO_FORM_CLUSTER = 3;
    constexpr int NUMBER_OF_DIMENSIONS = 3;

    // All points will have this data type
    using CoordinateType = double;

    // PointCloud point structure
    using Point = clustering::Point<
        CoordinateType, NUMBER_OF_DIMENSIONS
    >;

    try
    {
        clustering::PointCloud<
            CoordinateType, 
            NUMBER_OF_DIMENSIONS
        > point_cloud;

        // Load your point cloud into point_cloud here
        // ...

        clustering::DBSCAN<
            CoordinateType, 
            NUMBER_OF_DIMENSIONS
        > dbscan(
            SEARCH_RADIUS, 
            NUMBER_OF_POINTS_TO_FORM_CLUSTER, 
            point_cloud
        );

        // Call this method to form clusters
        dbscan.formClusters();

        // Get result indices of each cluster
        std::unordered_map<
            std::int32_t, 
            std::vector<std::int32_t>
        > clusters = dbscan.getClusterIndices();

        // Select only valid clusters without UNDEFINED labels
        // Container to hold clustered points
        std::vector<std::vector<Point>> point_cloud_clusters;
        point_cloud_cluster.reserve(clusters.size());

        // Iterate over cluster labels and indices (pairs)
        for (const auto& [cluster_label, cluster_indices] : clusters)
        {
            // Check if the clustered point is an outlier
            if (cluster_label == clustering::labels::UNDEFINED) 
            {
                continue;
            }
            // Prepare point cache for the current cluster
            std::vector<Point> point_cloud_cluster;
            point_cloud_cluster.reserve(cluster_indices.size());

            // Accumulate points into the point_cloud_cluster
            for (const auto& cluster_index : cluster_indices)
            {
                const Point& point_cache = point_cloud[cluster_index].point;
                point_cloud_cluster.push_back(point_cache);
            }

            // Append point cloud cluster to the list of clusters
            point_cloud_clusters.push_back(point_cloud_cluster);
        }
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch(...)
    {
        std::cerr << "Unknown exception" << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
```