#pragma once

#include "utilities/bounded_dynamic_array.hpp"
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <future>
#include <limits>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <queue>
#include <random>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <vector>

namespace lidar_processing
{
class ThreadPool
{
  private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex tasks_mutex;
    std::condition_variable condition;
    bool stop;

  public:
    ThreadPool(size_t threads) : stop(false)
    {
        for (size_t i = 0; i < threads; ++i)
        {
            workers.emplace_back([this]() {
                while (true)
                {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(tasks_mutex);
                        condition.wait(lock, [this] {
                            return stop || !tasks.empty();
                        });
                        if (stop && tasks.empty())
                        {
                            return;
                        }
                        task = std::move(tasks.front());
                        tasks.pop();
                    }
                    task();
                }
            });
        }
    }

    template <class F>
    auto enqueue(F&& f) -> std::future<decltype(f())>
    {
        using return_type = decltype(f());
        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::forward<F>(f));
        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(tasks_mutex);
            if (stop)
            {
                throw std::runtime_error("enqueue on stopped ThreadPool");
            }
            tasks.emplace([task]() {
                (*task)();
            });
        }
        condition.notify_one();
        return res;
    }

    ~ThreadPool()
    {
        {
            std::unique_lock<std::mutex> lock(tasks_mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread& worker : workers)
        {
            worker.join();
        }
    }
};

void segmentGroundRANSAC(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                         pcl::PointCloud<pcl::PointXYZRGBL>& ground_cloud,
                         pcl::PointCloud<pcl::PointXYZRGBL>& obstacle_cloud,
                         float orthogonal_distance_threshold = 0.1f,
                         float orthogonal_distance_post_convergence = 0.3f,
                         float probability_of_success = 0.999f,
                         float percentage_of_outliers = 0.65f,
                         std::uint32_t selected_points = 3U,
                         std::size_t thread_count = 8U)
{
    // Clear old buffers
    ground_cloud.clear();
    obstacle_cloud.clear();

    // Plane coefficients
    float a = 0.0f;
    float b = 0.0f;
    float c = 1.0f;
    float d = std::numeric_limits<float>::max();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<std::size_t> dis(
        0, input_cloud.points.size() - 1U);

    std::size_t indices[3];
    std::size_t best_inlier_count = 0U;

    std::uint32_t required_iterations = static_cast<std::uint32_t>(
        std::log(1.0f - probability_of_success) /
        std::log(1.0f -
                 std::pow((1.0f - percentage_of_outliers), selected_points)));

    std::cout << "Required iterations: " << required_iterations << std::endl;

    // Launch threads
    ThreadPool pool(thread_count);
    std::vector<std::future<std::size_t>> futures(thread_count);
    std::size_t chunk_size = input_cloud.points.size() / thread_count;

    for (std::size_t i = 0U; i < required_iterations; ++i)
    {
        // Choose 3 distinct random indices
        for (std::size_t k = 0U; k < 3U;)
        {
            indices[k] = dis(gen);
            if (std::find(indices, indices + k, indices[k]) == indices + k)
            {
                ++k;
            }
        }

        // Get random indices
        const auto& p1 = input_cloud.points[indices[0]];
        const auto& p2 = input_cloud.points[indices[1]];
        const auto& p3 = input_cloud.points[indices[2]];

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

        const float plane_d =
            normal_x * p1.x + normal_y * p1.y + normal_z * p1.z;

        // Parallel inlier counting
        for (std::size_t t = 0U; t < thread_count; ++t)
        {
            futures[t] = pool.enqueue([&, t]() {
                std::size_t local_count = 0U;
                for (std::size_t j = t * chunk_size;
                     j < (t + 1) * chunk_size && j < input_cloud.points.size();
                     j++)
                {
                    const auto& point = input_cloud.points[j];

                    const float distance =
                        std::fabs(normal_x * point.x + normal_y * point.y +
                                  normal_z * point.z - plane_d);

                    if (distance < orthogonal_distance_threshold)
                    {
                        ++local_count;
                    }
                }
                return local_count;
            });
        }

        // Collect results from all threads
        std::size_t inlier_count = 0U;
        for (std::size_t t = 0U; t < thread_count; ++t)
        {
            inlier_count += futures[t].get();
        }

        // If the plane is best so far, update the plane coefficients
        // This is under the constraint that the plane coefficients satisfy
        // angular constraints
        if (inlier_count > best_inlier_count)
        {
            best_inlier_count = inlier_count;
            a = normal_x;
            b = normal_y;
            c = normal_z;
            d = plane_d;
        }
    }

    // Using inliers, compute PCA
    if (best_inlier_count > 0U)
    {
        for (const auto& point : input_cloud.points)
        {
            if (std::fabs(a * point.x + b * point.y + c * point.z - d) <
                orthogonal_distance_post_convergence)
            {
                ground_cloud.points.emplace_back(point.x, point.y, point.z, 220,
                                                 220, 220, 0);
            }
            else
            {
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