#ifndef GROUND_SEGMENTATION
#define GROUND_SEGMENTATION

// BoundedVector
#include "utilities/bounded_dynamic_array.hpp"
#include "utilities/bounded_vector.hpp"

// STL
#include <algorithm>
#include <cstdint>
#include <execution>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <vector>

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

// PCL
#include <pcl/point_cloud.h> // pcl::PointCloud
#include <pcl/point_types.h> // pcl::PointXYZ

#define DEBUG 0

namespace lidar_processing
{
enum class SegmentationAlgorithm
{
    RANSAC,
    ITERATIVE_PLANE_FITTING,
    RULE_BASED_APPROACH
};

enum class SegmentationLabels
{
    GROUND = 0,
    OBSTACLE = 1
};

struct GroundPlane
{
    float a, b, c, d;
    explicit GroundPlane(const float &a_, const float &b_, const float &c_, const float &d_)
        : a(a_), b(b_), c(c_), d(d_){};
    ~GroundPlane() = default;
};

/// @brief Find plane from provided ground points. Plane equation: ax + by + cz
/// = d
/// @param points_xyz Point cloud matrix
/// @return Ground plane object containing plane coefficients (a, b, c, d)
GroundPlane estimatePlane(const Eigen::MatrixXf &points_xyz);

struct ColorRGB
{
    float r;
    float g;
    float b;
    ColorRGB(const float &r_, const float &g_, const float &b_) : r(r_), g(g_), b(b_){};
    ColorRGB() = default;
    ~ColorRGB() = default;
};

class GroundSegmenter final
{
  public:
    static constexpr std::size_t MAX_CLOUD_SIZE = 150'000;
    using BoundedDynamicArray = lidar_processing::utilities::BoundedDynamicArray<int, MAX_CLOUD_SIZE>;

    /// @brief Constructor of GroundSegmentation class
    /// @param number_of_iterations Fixed number of iterations for convergence
    /// @param number_of_planar_partitions Number of planar components to use
    /// for ground fitting
    /// @param number_of_lowest_point_representative_estimators Number of sample
    /// points to take for initial ground fit
    /// @param sensor_height Approximate height of a sensor
    /// @param distance_threshold Maximum distance from fitted plane to consider
    /// points a ground
    /// @param initial_seed_threshold Filter points that have height less than
    /// the height of the lowest point representative
    /// + initial_seed_threshold
    explicit GroundSegmenter(std::uint32_t number_of_iterations = 3, std::uint32_t number_of_planar_partitions = 2,
                             std::uint32_t number_of_lowest_point_representative_estimators = 5000,
                             float sensor_height = 1.73, float distance_threshold = 0.3,
                             float initial_seed_threshold = 0.6);

    ~GroundSegmenter() = default;

    template <typename PointT>
    void segmentGround(const typename pcl::PointCloud<PointT> &input_cloud,
                       pcl::PointCloud<pcl::PointXYZRGBL> &ground_cloud,
                       pcl::PointCloud<pcl::PointXYZRGBL> &obstacle_cloud);

    // Setters
    void setNumberOfIterations(const std::uint32_t &number_of_iterations)
    {
        number_of_iterations_ = number_of_iterations;
    }
    void setNumberOfPlanarPartitions(const std::uint32_t &number_of_planar_partitions)
    {
        number_of_planar_partitions_ = number_of_planar_partitions;
    }
    void setNumberOfLowestPointRepresentativeEstimators(
        const std::uint32_t &number_of_lowest_point_representative_estimators)
    {
        number_of_lowest_point_representative_estimators_ = number_of_lowest_point_representative_estimators;
    }
    void setSensorHeight(const float &sensor_height)
    {
        sensor_height_ = sensor_height;
    }
    void setDistanceThreshold(const float &distance_threshold)
    {
        distance_threshold_ = distance_threshold;
    }
    void setInitialSeedThreshold(const float &initial_seed_threshold)
    {
        initial_seed_threshold_ = initial_seed_threshold;
    }

    // Getters
    const std::uint32_t &getNumberOfIterations() const
    {
        return number_of_iterations_;
    }
    const std::uint32_t &getNumberOfPlanarPartitions() const
    {
        return number_of_planar_partitions_;
    }
    const std::uint32_t &getNumberOfLowestPointRepresentativeEstimators() const
    {
        return number_of_lowest_point_representative_estimators_;
    }
    const float &getSensorHeight() const
    {
        return sensor_height_;
    }
    const float &getDistanceThreshold() const
    {
        return distance_threshold_;
    }
    const float &getInitialSeedThreshold() const
    {
        return initial_seed_threshold_;
    }

  private:
    /// @brief Partitions space into multiple planar components
    /// @param cloud Input point cloud to be split into multiple planes
    /// @param cloud_segments Vector of point cloud components
    void formPlanarPartitions(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                              std::vector<pcl::PointCloud<pcl::PointXYZ>> &cloud_segments);

    /// @brief Initial estimation of ground points
    /// @param cloud Input point cloud
    /// @param indices Indices corresponding to ground plane points
    void extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZ> &cloud, BoundedDynamicArray &indices);

    /// @brief Ground segmentation for a planar partition
    /// @param cloud_segment Pointcloud partition extracted from
    /// formPlanarPartitions() method
    /// @param ground_cloud_segment Ground cloud points for current partition
    /// @param obstacle_cloud_segment Obstacle cloud points for current
    /// partition
    void fitGroundPlane(const pcl::PointCloud<pcl::PointXYZ> &cloud_segment,
                        pcl::PointCloud<pcl::PointXYZ> &ground_cloud_segment,
                        pcl::PointCloud<pcl::PointXYZ> &obstacle_cloud_segment);

    /// @brief Combines ground of obstacle points from multiple partitions
    /// @param cloud_segments Point cloud corresponding to either ground or
    /// obstacle cloud partition
    /// @param segmentation_label Segmentation label to be assigned to this
    /// point cloud
    /// @param segmented_cloud Output point cloud with segmentation labels
    void combineSegmentedPoints(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &cloud_segments,
                                const SegmentationLabels &segmentation_label,
                                pcl::PointCloud<pcl::PointXYZRGBL> &segmented_cloud) const;

    std::uint32_t number_of_iterations_;
    std::uint32_t number_of_planar_partitions_;
    std::uint32_t number_of_lowest_point_representative_estimators_;
    float sensor_height_;
    float distance_threshold_;
    float initial_seed_threshold_;
};

/// @brief Performs ground segmentation on input cloud
/// @tparam PointT Point type - struct containing x, y, z fields
/// @param input_cloud Input point cloud
/// @param ground_cloud Segmented ground point cloud
/// @param obstacle_cloud Segmented obstacle point cloud
template <typename PointT>
void GroundSegmenter::segmentGround(const typename pcl::PointCloud<PointT> &input_cloud,
                                    pcl::PointCloud<pcl::PointXYZRGBL> &ground_cloud,
                                    pcl::PointCloud<pcl::PointXYZRGBL> &obstacle_cloud)
{
    ground_cloud.clear();
    obstacle_cloud.clear();

    if (input_cloud.empty())
    {
        return;
    }

    const auto &number_of_points = input_cloud.points.size();

    // convert input pointcloud from PointT to PointXYZ
    auto converted_input_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    for (const auto &point : input_cloud.points)
    {
        converted_input_cloud->points.emplace_back(point.x, point.y, point.z);
    }
    converted_input_cloud->width = converted_input_cloud->points.size();
    converted_input_cloud->height = 1;
    converted_input_cloud->is_dense = true;

    // partition point cloud into segments
    std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_segments;
    formPlanarPartitions(*converted_input_cloud, cloud_segments);

    // iterate over segments
    std::vector<pcl::PointCloud<pcl::PointXYZ>> ground_cloud_segments;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacle_cloud_segments;

    for (const auto &cloud_segment : cloud_segments)
    {
        auto ground_cloud_segment = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
        auto obstacle_cloud_segment = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();

        // segment points into ground and non-ground points
        fitGroundPlane(cloud_segment, *ground_cloud_segment, *obstacle_cloud_segment);

        // accumulate into corresponding vectors
        ground_cloud_segments.push_back(std::move(*ground_cloud_segment));
        obstacle_cloud_segments.push_back(std::move(*obstacle_cloud_segment));

        // Clean underlying pointers
        ground_cloud_segment.reset();
        obstacle_cloud_segment.reset();
    }

    combineSegmentedPoints(ground_cloud_segments, SegmentationLabels::GROUND, ground_cloud);
    combineSegmentedPoints(obstacle_cloud_segments, SegmentationLabels::OBSTACLE, obstacle_cloud);
}

} // namespace lidar_processing

#endif // GROUND_SEGMENTATION