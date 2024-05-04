/*
 * Copyright (c) 2024 Yevgeniy Simonov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "kdtree.hpp"

#include <gtest/gtest.h>

#include <random>

using namespace lidar_processing;

TEST(KDTreeTest, KnnMatchesBruteForce)
{
    constexpr std::size_t NUM_PTS = 1000UL;
    constexpr std::size_t NUM_TEST_PTS = 50UL;
    constexpr std::size_t NUM_DIM = 3UL;
    constexpr std::uint32_t NUM_NEIGH = 5U;

    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_real_distribution<double> dist(-10.0, 10.0);

    std::vector<Point<double, NUM_DIM>> points;
    points.reserve(NUM_PTS);
    for (std::size_t i = 0UL; i < NUM_PTS; ++i)
    {
        points.push_back({dist(gen), dist(gen), dist(gen)});
    }

    std::vector<Point<double, NUM_DIM>> test_points;
    test_points.reserve(NUM_TEST_PTS);
    for (std::size_t i = 0UL; i < NUM_TEST_PTS; ++i)
    {
        test_points.push_back({dist(gen), dist(gen), dist(gen)});
    }

    KDTree<double, NUM_DIM> kdtree(true);
    kdtree.reserve(points.size());
    kdtree.rebuild(points);

    containers::Vector<decltype(kdtree)::RetT> kd_neigh;
    containers::Vector<decltype(kdtree)::RetT> brute_force_results;

    for (const auto &test_point : test_points)
    {
        // Find k neighbours
        kdtree.k_nearest(test_point, NUM_NEIGH, kd_neigh);

        // Brute-force nearest neighbor search
        brute_force_results.clear();
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            double dist = 0.0;
            for (std::size_t dim = 0; dim < NUM_DIM; ++dim)
            {
                double delta = points[i][dim] - test_point[dim];
                dist += delta * delta;
            }
            brute_force_results.emplace_back(i, dist);
        }

        // Sort brute-force results by distance
        std::sort(brute_force_results.begin(), brute_force_results.end(),
                  [](const auto &a, const auto &b) { return a.second < b.second; });

        // Ensure KDTree results are sorted and match the brute-force results
        std::sort(kd_neigh.begin(), kd_neigh.end(), [](const auto &a, const auto &b) { return a.second < b.second; });

        ASSERT_LE(kd_neigh.size(), NUM_NEIGH);
        for (std::size_t i = 0; i < kd_neigh.size(); ++i)
        {
            ASSERT_EQ(kd_neigh[i].first, brute_force_results[i].first);
            ASSERT_NEAR(kd_neigh[i].second, brute_force_results[i].second, 1e-12);
        }
    }
}

TEST(KDTreeTest, RadiusSearchMatchesBruteForce)
{
    constexpr std::size_t NUM_PTS = 1000UL;
    constexpr std::size_t NUM_TEST_PTS = 50UL;
    constexpr std::size_t NUM_DIM = 3UL;
    constexpr double SEARCH_RADIUS = 2.0;
    constexpr double SEARCH_RADIUS_SQUARED = SEARCH_RADIUS * SEARCH_RADIUS;

    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_real_distribution<double> dist(-15.0, 15.0);

    std::vector<Point<double, NUM_DIM>> points;
    points.reserve(NUM_PTS);
    for (std::size_t i = 0UL; i < NUM_PTS; ++i)
    {
        points.push_back({dist(gen), dist(gen), dist(gen)});
    }

    std::vector<Point<double, NUM_DIM>> test_points;
    test_points.reserve(NUM_TEST_PTS);
    for (std::size_t i = 0UL; i < NUM_TEST_PTS; ++i)
    {
        test_points.push_back({dist(gen), dist(gen), dist(gen)});
    }

    KDTree<double, NUM_DIM> kdtree(true);
    kdtree.reserve(points.size());
    kdtree.rebuild(points);

    containers::Vector<decltype(kdtree)::RetT> neighbour_indices_and_distances_squared;
    neighbour_indices_and_distances_squared.reserve(NUM_PTS);

    for (const auto &test_point : test_points)
    {
        // Find all points within specific radius
        neighbour_indices_and_distances_squared.clear();
        kdtree.radius_search(test_point, SEARCH_RADIUS_SQUARED, neighbour_indices_and_distances_squared);

        // Find neighbours within radius using brute force
        std::vector<Point<double, NUM_DIM>> neighbours_brute_force;
        std::vector<double> distances_brute_force;
        for (const auto &point : points)
        {
            double dist_sqr = 0.0;
            for (std::size_t dim = 0; dim < NUM_DIM; ++dim)
            {
                double delta = point[dim] - test_point[dim];
                dist_sqr += delta * delta;
            }
            if (dist_sqr != 0.0 && dist_sqr < SEARCH_RADIUS_SQUARED)
            {
                neighbours_brute_force.emplace_back(point);
                distances_brute_force.emplace_back(dist_sqr);
            }
        }

        // Check that the number of points matches
        ASSERT_EQ(neighbours_brute_force.size(), neighbour_indices_and_distances_squared.size());

        // Sort points in ascending order and check that points match
        std::vector<std::size_t> indices_brute_force;
        indices_brute_force.reserve(neighbours_brute_force.size());
        for (std::size_t i = 0; i < neighbours_brute_force.size(); ++i)
        {
            indices_brute_force.emplace_back(i);
        }

        std::sort(indices_brute_force.begin(), indices_brute_force.end(),
                  [&distances_brute_force](const std::size_t &idx_1, const std::size_t &idx_2) {
                      return (distances_brute_force[idx_1] < distances_brute_force[idx_2]);
                  });

        std::vector<Point<double, NUM_DIM>> neighbours;
        neighbours.reserve(neighbours_brute_force.size());

        std::vector<double> distances;
        distances.reserve(distances_brute_force.size());

        for (std::size_t i = 0; i < indices_brute_force.size(); ++i)
        {
            const std::size_t &index = indices_brute_force[i];
            for (int dim = 0; dim < NUM_DIM; ++dim)
            {
                ASSERT_DOUBLE_EQ(neighbours_brute_force[index][dim],
                                 points[neighbour_indices_and_distances_squared[i].first][dim]);
            }
            ASSERT_DOUBLE_EQ(distances_brute_force[index], neighbour_indices_and_distances_squared[i].second);
        }
    }
}
