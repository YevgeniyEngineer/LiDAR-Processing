#pragma once

#include "utilities/bounded_dynamic_array.hpp"
#include <cmath>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <random>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace lidar_processing
{
void segmentGroundRANSAC(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                         pcl::PointCloud<pcl::PointXYZRGBL>& ground_cloud,
                         pcl::PointCloud<pcl::PointXYZRGBL>& obstacle_cloud,
                         float orthogonal_distance_threshold = 0.3f,
                         float probability_of_success = 0.99f,
                         float percentage_of_outliers = 0.65f,
                         std::uint32_t selected_points = 3U)
{
    // Clear old buffers
    ground_cloud.clear();
    obstacle_cloud.clear();

    // Plane coefficients
    float a = 0.0f;
    float b = 0.0f;
    float c = 1.0f;
    float d = std::numeric_limits<float>::infinity();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, input_cloud.points.size() - 1);

    std::unordered_set<std::int64_t> indices;
    indices.reserve(3);

    std::size_t best_inlier_count = 0U;

    std::vector<std::size_t> best_inliers;
    best_inliers.reserve(input_cloud.points.size());

    std::vector<std::size_t> current_inliers;
    current_inliers.reserve(input_cloud.points.size());

    std::uint32_t required_iterations = static_cast<std::uint32_t>(
        std::log(1.0f - probability_of_success) /
        std::log(1.0f -
                 std::pow((1.0f - percentage_of_outliers), selected_points)));

    std::cout << "Required iterations: " << required_iterations << std::endl;

    for (std::size_t i = 0U; i < required_iterations; ++i)
    {

        // Choose 3 random indices
        indices.clear();
        while (indices.size() < 3)
        {
            indices.insert(dis(gen));
        }

        // Get random indices
        auto it = indices.cbegin();
        const auto& p1 = input_cloud.points[*it];
        ++it;
        const auto& p2 = input_cloud.points[*it];
        ++it;
        const auto& p3 = input_cloud.points[*it];

        // Calculate a plane defined by three points
        float normal_x =
            ((p2.y - p1.y) * (p3.z - p1.z)) - ((p2.z - p1.z) * (p3.y - p1.y));
        float normal_y =
            ((p2.z - p1.z) * (p3.x - p1.x)) - ((p2.x - p1.x) * (p3.z - p1.z));
        float normal_z =
            ((p2.x - p1.x) * (p3.y - p1.y)) - ((p2.y - p1.y) * (p3.x - p1.x));

        // normalize plane coefficients
        const float normalization =
            1.0f / std::sqrt(normal_x * normal_x + normal_y * normal_y +
                             normal_z * normal_z);

        // normalize
        normal_x *= normalization;
        normal_y *= normalization;
        normal_z *= normalization;

        float plane_d = normal_x * p1.x + normal_y * p1.y + normal_z * p1.z;

        // Count the points close to the plane based on orthogonal distances
        current_inliers.clear();
        for (std::size_t j = 0U; j < input_cloud.points.size(); ++j)
        {
            const auto& point = input_cloud.points[j];

            const float orthogonal_distance =
                std::fabs(normal_x * point.x + normal_y * point.y +
                          normal_z * point.z - plane_d);

            if (orthogonal_distance < orthogonal_distance_threshold)
            {
                current_inliers.push_back(j);
            }
        }

        // If the plane is best so far, update the plane coefficients
        // This is under the constraint that the plane coefficients satisfy
        // angular constraints
        if (current_inliers.size() > best_inlier_count)
        {
            best_inlier_count = current_inliers.size();
            a = normal_x;
            b = normal_y;
            c = normal_z;
            d = plane_d;
            best_inliers.swap(current_inliers);
        }
    }

    // Using inliers, compute PCA
    if (best_inlier_count > 0U)
    {
        // Split points into ground and obstacle sets
        ground_cloud.reserve(best_inliers.size());
        std::vector<bool> is_ground(input_cloud.points.size(), false);
        for (std::size_t index : best_inliers)
        {
            is_ground[index] = true;
            const auto& point = input_cloud.points[index];
            ground_cloud.points.emplace_back(point.x, point.y, point.z, 220,
                                             220, 220, 0);
        }

        obstacle_cloud.reserve(input_cloud.points.size() - best_inliers.size());
        for (std::size_t i = 0; i < input_cloud.points.size(); ++i)
        {
            if (!is_ground[i])
            {
                const auto& point = input_cloud.points[i];
                obstacle_cloud.points.emplace_back(point.x, point.y, point.z, 0,
                                                   255, 0, 1);
            }
        }

        ground_cloud.width = ground_cloud.size();
        ground_cloud.height = 1;

        obstacle_cloud.width = obstacle_cloud.size();
        obstacle_cloud.height = 1;
    }
}
}; // namespace lidar_processing