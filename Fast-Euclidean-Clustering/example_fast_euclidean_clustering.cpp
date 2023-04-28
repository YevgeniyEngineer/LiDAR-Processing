
#include "fec_clustering.hpp"

#include <chrono>
#include <iostream>
#include <limits>
#include <random>

int main()
{
    constexpr std::size_t NUMBER_OF_POINTS = 100'000;
    constexpr std::size_t NUMBER_OF_DIMENSIONS = 3;
    constexpr std::size_t NUMBER_OF_ITERATIONS = 100;
    constexpr std::size_t MIN_CLUSTER_SIZE = 3;
    constexpr std::size_t MAX_CLUSTER_SIZE = std::numeric_limits<std::size_t>::max();

    constexpr double NEAREST_NEIGHBOUR_PROXIMITY = 8.0;

    using CoordinateType = double;
    using PointType = clustering::FECPoint<CoordinateType, NUMBER_OF_DIMENSIONS>;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<CoordinateType> dist(-100.0, 100.0);

    std::vector<PointType> points;

    clustering::FECPointCloud<CoordinateType, NUMBER_OF_DIMENSIONS> cloud;
    cloud.reserve(NUMBER_OF_POINTS);
    for (auto i = 0; i < NUMBER_OF_POINTS; ++i)
    {
        PointType point_cache;
        for (auto j = 0; j < NUMBER_OF_DIMENSIONS; ++j)
        {
            point_cache[j] = dist(gen);
        }
        cloud.push_back(point_cache);
    }

    auto t1 = std::chrono::steady_clock::now();
    {
        clustering::FECClustering<CoordinateType, NUMBER_OF_DIMENSIONS> fast_euclidean_clustering(cloud);
        fast_euclidean_clustering.clusterTolerance(NEAREST_NEIGHBOUR_PROXIMITY);
        fast_euclidean_clustering.minClusterSize(MIN_CLUSTER_SIZE);
        fast_euclidean_clustering.quality(1.0);

        fast_euclidean_clustering.formClusters();

        auto cluster_indices = fast_euclidean_clustering.getClusterIndices();

        std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;
    }
    auto t2 = std::chrono::steady_clock::now();
    std::cout << "Elapsed time (s): " << (t2 - t1).count() / 1e9 << std::endl;

    return EXIT_SUCCESS;
}