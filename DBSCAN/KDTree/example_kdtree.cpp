#include "kdtree.hpp"
#include <chrono>
#include <cstdint>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

using CoordinateType = double;
constexpr std::size_t NUMBER_OF_DIMENSIONS = 3;
constexpr std::size_t NUMBER_OF_POINTS = 100000;
constexpr std::size_t NUMBER_OF_NEAREST_NEIGHBORS = 20;
constexpr CoordinateType RADIUS_SQUARED = 1.0;

void generateRandomPoints(std::vector<neighbour_search::Point<CoordinateType, NUMBER_OF_DIMENSIONS>> &points)
{
    points.clear();
    points.reserve(NUMBER_OF_POINTS);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<CoordinateType> dist(-10.0, 10.0);

    for (std::size_t i = 0; i < NUMBER_OF_POINTS; ++i)
    {
        neighbour_search::Point<CoordinateType, NUMBER_OF_DIMENSIONS> point{dist(gen), dist(gen), dist(gen)};
        points.push_back(point);
    }
}

std::int32_t main(std::int32_t argc, const char **const argv)
{
    try
    {
        std::vector<neighbour_search::Point<CoordinateType, NUMBER_OF_DIMENSIONS>> points;
        generateRandomPoints(points);

        auto t1 = std::chrono::high_resolution_clock::now();

        neighbour_search::KDTree kdtree(points, true);

        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "KDTree construction time (s): " << (t2 - t1).count() / 1.0e9 << std::endl;

        neighbour_search::Point<CoordinateType, NUMBER_OF_DIMENSIONS> target{0.2, -1.3, 2.5};
        auto t3 = std::chrono::high_resolution_clock::now();

        auto nearest_point = kdtree.findNearestNeighbour(target);

        auto t4 = std::chrono::high_resolution_clock::now();
        std::cout << "KDTree nearest neighbour search time (s): " << (t4 - t3).count() / 1.0e9
                  << ", neighbour index: " << nearest_point.first
                  << ", neighbour distance squared: " << nearest_point.second << std::endl;

        auto t5 = std::chrono::high_resolution_clock::now();

        std::vector<std::pair<std::size_t, CoordinateType>> knn_result;
        kdtree.findKNearestNeighbours(target, NUMBER_OF_NEAREST_NEIGHBORS, knn_result);

        auto t6 = std::chrono::high_resolution_clock::now();
        std::cout << "KDTree nearest K = " << NUMBER_OF_NEAREST_NEIGHBORS
                  << " neighbour search time (s): " << (t6 - t5).count() / 1.0e9
                  << ", total neighbours found: " << knn_result.size() << std::endl;

        // for (const auto &[index, distance] : knn_result)
        // {
        //     std::cout << "Neighbor: " << index << " Distance: " << distance << "\n";
        // }

        auto t7 = std::chrono::high_resolution_clock::now();

        std::vector<std::pair<std::size_t, CoordinateType>> knn_within_radius_result;

        kdtree.findKNearestNeighboursWithinRadiusSquared(target, NUMBER_OF_NEAREST_NEIGHBORS, RADIUS_SQUARED,
                                                         knn_within_radius_result);

        auto t8 = std::chrono::high_resolution_clock::now();
        std::cout << "KDTree nearest K = " << NUMBER_OF_NEAREST_NEIGHBORS
                  << " neighbour search within radius squared of " << RADIUS_SQUARED
                  << ", time (s): " << (t8 - t7).count() / 1.0e9
                  << ", total neighbours found: " << knn_within_radius_result.size() << std::endl;

        // for (const auto &[index, distance] : knn_within_radius_result)
        // {
        //     std::cout << "Neighbor: " << index << " Distance: " << distance << "\n";
        // }

        auto t9 = std::chrono::high_resolution_clock::now();

        std::vector<std::pair<std::size_t, CoordinateType>> nn_within_radius_result;
        nn_within_radius_result.reserve(1000);

        kdtree.findAllNearestNeighboursWithinRadiusSquared(target, RADIUS_SQUARED, nn_within_radius_result);

        auto t10 = std::chrono::high_resolution_clock::now();
        std::cout << "KDTree nearest neighbour search within radius squared of " << RADIUS_SQUARED
                  << ", time (s): " << (t10 - t9).count() / 1.0e9
                  << ", total neighbours found: " << nn_within_radius_result.size() << std::endl;

        // for (const auto &[index, distance] : nn_within_radius_result)
        // {
        //     std::cout << "Neighbor: " << index << " Distance: " << distance << "\n";
        // }
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (...)
    {
        std::cerr << "Unknown exception: " << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}