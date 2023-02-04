#ifndef GROUND_SEGMENTATION
#define GROUND_SEGMENTATION

// STL
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <execution>
#include <memory>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

// Eigen
#include <eigen3/Eigen/Dense>

// PCL
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#define DEBUG 0

namespace lidar_processing
{
class GroundSegmenter
{
  public:
    explicit GroundSegmenter(std::uint32_t number_of_iterations = 3, std::uint32_t number_of_planar_partitions = 2,
                             std::uint32_t number_of_lowest_point_representative_estimators = 400,
                             float sensor_height = 1.73, float distance_threshold = 0.3,
                             float initial_seed_threshold = 0.6)
        : number_of_iterations_(number_of_iterations), number_of_planar_partitions_(number_of_planar_partitions),
          number_of_lowest_point_representative_estimators_(number_of_lowest_point_representative_estimators),
          sensor_height_(sensor_height), distance_threshold_(distance_threshold),
          initial_seed_threshold_(initial_seed_threshold)
    {
        if (number_of_iterations <= 0)
        {
            throw std::runtime_error("Number of iterations must be greater than 0");
        }
        if (number_of_planar_partitions <= 0)
        {
            throw std::runtime_error("Number of planar partitions must be greater than 0");
        }
        if (number_of_lowest_point_representative_estimators < 3)
        {
            throw std::runtime_error("Number of lowest point representative estimators must contain at least 3 points");
        }
        if (distance_threshold < 0)
        {
            throw std::runtime_error("Distance threshold must be non-negative");
        }
        if (initial_seed_threshold < 0)
        {
            throw std::runtime_error("Initial seed threshold must be non-negative");
        }
    };

    virtual ~GroundSegmenter() = default;

    template <typename PointT>
    void segmentGround(const pcl::PointCloud<PointT> &input_cloud, pcl::PointCloud<pcl::PointXYZRGBL> &ground_cloud,
                       pcl::PointCloud<pcl::PointXYZRGBL> &obstacle_cloud);

  private:
    template <typename PointT>
    void formPlanarPartitions(const pcl::PointCloud<PointT> &cloud,
                              std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_segments);

    void extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<int> &indices);

    void fitGroundPlane(const pcl::PointCloud<pcl::PointXYZ> &cloud_segment,
                        pcl::PointCloud<pcl::PointXYZRGBL> &ground_cloud_segment,
                        pcl::PointCloud<pcl::PointXYZRGBL> &obstacle_cloud_segment);

  private:
    const std::uint32_t number_of_iterations_;
    const std::uint32_t number_of_planar_partitions_;
    const std::uint32_t number_of_lowest_point_representative_estimators_;
    const float sensor_height_;
    const float distance_threshold_;
    const float initial_seed_threshold_;
};

// partitions space into multiple planar components
template <typename PointT>
void GroundSegmenter::formPlanarPartitions(const pcl::PointCloud<PointT> &cloud,
                                           std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_segments)
{
    if (cloud.empty())
    {
        return;
    }

    const auto &number_of_points = cloud.size();

    // sort and get indices corresponding to the point cloud that is sorted in increasing x-order
    std::vector<size_t> sorted_indices(number_of_points);

    // fill vector with increasing values, starting from 0
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

    // sort indices using stable sort
    const auto &cloud_points = cloud.points;
    std::sort(std::execution::par, sorted_indices.begin(), sorted_indices.end(),
              [&cloud_points](const size_t &idx_1, const size_t &idx_2) -> bool {
                  return cloud_points[idx_1].x < cloud_points[idx_2].x;
              });

    // iterate over sorted indices and partition point cloud
    size_t elements_within_segment = number_of_points / number_of_planar_partitions_;
    size_t idx_low = 0;
    size_t idx_high = elements_within_segment;
    cloud_segments.reserve(number_of_planar_partitions_);

    pcl::PointXYZ point_cache;
    for (size_t segment_no = 0; segment_no < number_of_planar_partitions_; ++segment_no)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segment = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud_segment->points.reserve(idx_high - idx_low + 1);

        for (size_t sorted_idx_no = idx_low; sorted_idx_no < idx_high; ++sorted_idx_no)
        {
            const auto &cloud_point = cloud_points[sorted_indices[sorted_idx_no]];

            point_cache.x = cloud_point.x;
            point_cache.y = cloud_point.y;
            point_cache.z = cloud_point.z;

            cloud_segment->points.emplace_back(point_cache);
        }

        cloud_segments.emplace_back(std::move(cloud_segment));

        // update indices
        idx_low = idx_high;
        idx_high = std::min(idx_low + elements_within_segment, number_of_points);
    }
}

// initial estimation of ground points
void GroundSegmenter::extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<int> &indices)
{
    if (cloud.empty())
    {
        return;
    }

    const auto &number_of_points = cloud.points.size();

    // copy points in cloud copy
    std::vector<pcl::PointXYZ> cloud_copy(cloud.points.cbegin(), cloud.points.cend());

    // get indices of a sorted array
    std::vector<int> cloud_indices(number_of_points);
    std::iota(cloud_indices.begin(), cloud_indices.end(), 0);

    // apply sorting on copied point cloud
    std::sort(std::execution::par, cloud_indices.begin(), cloud_indices.end(),
              [&](const int &index_1, const int &index_2) -> bool {
                  return (cloud_copy[index_1].z < cloud_copy[index_2].z);
              });

    // remove outlier points located below ground
    float negative_offset_threshold = -1.5 * sensor_height_;
    int lower_cutoff_index = 0;
    for (int i = 0; i < cloud_indices.size(); ++i)
    {
        if (cloud.points[cloud_indices[i]].z > negative_offset_threshold)
        {
            lower_cutoff_index = i;
            break;
        }
    }
    if (lower_cutoff_index == 0)
    {
        return;
    }
    cloud_indices.erase(cloud_indices.begin(), cloud_indices.begin() + lower_cutoff_index);

    // find the average height of the lowest point representatives
    float lowest_point_representative_height = 0.0F;
    int number_of_estimators = std::min(static_cast<int>(cloud_indices.size()),
                                        static_cast<int>(number_of_lowest_point_representative_estimators_));
    for (int i = 0; i < number_of_estimators; ++i)
    {
        lowest_point_representative_height += cloud.points[cloud_indices[i]].z;
    }
    lowest_point_representative_height /= number_of_estimators;

    // filter points that have height less that lowest_point_representative_height + initial_seed_threshold_
    float cutoff_height = lowest_point_representative_height + initial_seed_threshold_;
    int upper_cutoff_index = 0;
    for (int i = 0; i < cloud_indices.size(); ++i)
    {
        if (cloud.points[cloud_indices[i]].z > cutoff_height)
        {
            upper_cutoff_index = i;
            break;
        }
    }

    // move indices
    indices.clear();
    indices.insert(indices.end(), std::make_move_iterator(cloud_indices.begin()),
                   std::make_move_iterator(cloud_indices.begin() + upper_cutoff_index));

#if DEBUG
    std::cout << "Number of seed points: " << ground_indices.points.size() << std::endl;
#endif
}

void GroundSegmenter::fitGroundPlane(const pcl::PointCloud<pcl::PointXYZ> &cloud_segment,
                                     pcl::PointCloud<pcl::PointXYZRGBL> &ground_cloud_segment,
                                     pcl::PointCloud<pcl::PointXYZRGBL> &obstacle_cloud_segment)
{
}

template <typename PointT>
void GroundSegmenter::segmentGround(const pcl::PointCloud<PointT> &input_cloud,
                                    pcl::PointCloud<pcl::PointXYZRGBL> &ground_cloud,
                                    pcl::PointCloud<pcl::PointXYZRGBL> &obstacle_cloud)
{
}

} // namespace lidar_processing

#endif // GROUND_SEGMENTATION