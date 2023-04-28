#include "convex_hull.hpp"

#include <chrono>
#include <iostream>
#include <random>

int main()
{
    using namespace geom;
    using PointType = double;

    int num_pts = 99'999;
    int coords_range = 1000;
    bool print_results = true;
    auto orientation = Orientation::CLOCKWISE;

    // Seed the random number generator with the current time
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 generator(seed);
    std::uniform_real_distribution<PointType> distribution(0, coords_range);

    // Generate random points
    std::vector<Point<PointType>> points;
    points.reserve(num_pts);
    for (int i = 0; i < num_pts; ++i)
    {
        points.push_back(Point<PointType>(distribution(generator), distribution(generator)));
    }

    auto printConvexHullIndices = [](const std::vector<int> &indices, bool print) {
        if (print)
        {
            std::cout << "Convex Hull Indices:" << std::endl;
            for (int index : indices)
            {
                std::cout << index << " ";
            }
            std::cout << std::endl;
        }
    };

    auto printHullOrientation = [](const std::vector<Point<PointType>> &points, const std::vector<int> &indices) {
        if (indices.size() < 3)
        {
            return;
        }
        // Check if points are counterclockwise
        auto p1 = points[indices[0]];
        auto p2 = points[indices[1]];
        auto p3 = points[indices[2]];

        auto orientation = getOrientation(p1, p2, p3);

        if (orientation == Orientation::COUNTERCLOCKWISE)
        {
            std::cout << "Hull is oriented counterclockwise" << std::endl;
        }
        else if (orientation == Orientation::CLOCKWISE)
        {
            std::cout << "Hull is oriented clockwise" << std::endl;
        }
    };

    // Print number of points before starting
    std::cout << "Generating convex hull from " << num_pts << " points" << std::endl;

    // Construct convex hull using GRAHAM SCAN
    {
        std::cout << std::endl << "Constructing convex hull using GRAHAM SCAN " << std::endl;

        auto t1 = std::chrono::high_resolution_clock::now();

        auto indices = constructConvexHull(points, ConvexHullAlgorithm::GRAHAM_SCAN, orientation);

        auto t2 = std::chrono::high_resolution_clock::now();

        std::cout << "Elapsed time (s): " << (t2 - t1).count() / 1e9 << std::endl;
        std::cout << "Number of hull points: " << indices.size() << std::endl;

        // Print hull orientation
        printHullOrientation(points, indices);

        // Print hull indices
        printConvexHullIndices(indices, print_results);
    }

    // Construct convex hull using ANDREW MONOTONE CHAIN
    {
        std::cout << std::endl << "Constructing convex hull using ANDREW MONOTONE CHAIN " << std::endl;

        auto t1 = std::chrono::high_resolution_clock::now();

        auto indices = constructConvexHull(points, ConvexHullAlgorithm::ANDREW_MONOTONE_CHAIN, orientation);

        auto t2 = std::chrono::high_resolution_clock::now();

        std::cout << "Elapsed time (s): " << (t2 - t1).count() / 1e9 << std::endl;
        std::cout << "Number of hull points: " << indices.size() << std::endl;

        // Print hull orientation
        printHullOrientation(points, indices);

        // Print hull indices
        printConvexHullIndices(indices, print_results);
    }

    // Construct convex hull using JARVIS MARCH
    {
        std::cout << std::endl << "Constructing convex hull using JARVIS MARCH " << std::endl;

        auto t1 = std::chrono::high_resolution_clock::now();

        auto indices = constructConvexHull(points, ConvexHullAlgorithm::JARVIS_MARCH, orientation);

        auto t2 = std::chrono::high_resolution_clock::now();

        std::cout << "Elapsed time (s): " << (t2 - t1).count() / 1e9 << std::endl;
        std::cout << "Number of hull points: " << indices.size() << std::endl;

        // Print hull orientation
        printHullOrientation(points, indices);

        // Print hull indices
        printConvexHullIndices(indices, print_results);
    }

    // Construct convex hull using CHAN's algorithm
    {
        std::cout << std::endl << "Constructing convex hull using CHAN " << std::endl;

        auto t1 = std::chrono::high_resolution_clock::now();

        auto indices = constructConvexHull(points, ConvexHullAlgorithm::CHAN, orientation);

        auto t2 = std::chrono::high_resolution_clock::now();

        std::cout << "Elapsed time (s): " << (t2 - t1).count() / 1e9 << std::endl;
        std::cout << "Number of hull points: " << indices.size() << std::endl;

        // Print hull orientation
        printHullOrientation(points, indices);

        // Print hull indices
        printConvexHullIndices(indices, print_results);
    }

    return 0;
}