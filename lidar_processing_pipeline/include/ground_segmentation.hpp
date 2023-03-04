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
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>

// PCL
#include <pcl/point_cloud.h> // pcl::PointCloud
#include <pcl/point_types.h> // pcl::PointXYZ

#define DEBUG 0

namespace lidar_processing
{
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

struct ColorRGB
{
    float r;
    float g;
    float b;
    ColorRGB(const float &r_, const float &g_, const float &b_) : r(r_), g(g_), b(b_){};
    ColorRGB() = default;
    ~ColorRGB() = default;
};

class GroundSegmenter
{
  public:
    GroundSegmenter(std::uint32_t number_of_iterations = 3, std::uint32_t number_of_planar_partitions = 1,
                    std::uint32_t number_of_lowest_point_representative_estimators = 400, float sensor_height = 1.73,
                    float distance_threshold = 0.3, float initial_seed_threshold = 0.6);

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
    void formPlanarPartitions(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                              std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_segments);

    void extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<int> &indices);

    void fitGroundPlane(const pcl::PointCloud<pcl::PointXYZ> &cloud_segment,
                        pcl::PointCloud<pcl::PointXYZ> &ground_cloud_segment,
                        pcl::PointCloud<pcl::PointXYZ> &obstacle_cloud_segment);

    void combineSegmentedPoints(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_segments,
                                const SegmentationLabels &segmentation_label,
                                pcl::PointCloud<pcl::PointXYZRGBL> &segmented_cloud) const;

  private:
    std::uint32_t number_of_iterations_;
    std::uint32_t number_of_planar_partitions_;
    std::uint32_t number_of_lowest_point_representative_estimators_;
    float sensor_height_;
    float distance_threshold_;
    float initial_seed_threshold_;
};

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
    auto converted_input_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (const auto &point : input_cloud.points)
    {
        converted_input_cloud->points.emplace_back(point.x, point.y, point.z);
    }
    converted_input_cloud->width = converted_input_cloud->points.size();
    converted_input_cloud->height = 1;
    converted_input_cloud->is_dense = true;

    // partition point cloud into segments
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments;
    formPlanarPartitions(*converted_input_cloud, cloud_segments);

    // iterate over segments
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ground_cloud_segments;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacle_cloud_segments;

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_segment : cloud_segments)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_segment = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_segment = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        // segment points into ground and non-ground points
        fitGroundPlane(*cloud_segment, *ground_cloud_segment, *obstacle_cloud_segment);

        // accumulate into corresponding vectors
        ground_cloud_segments.emplace_back(std::move(ground_cloud_segment));
        obstacle_cloud_segments.emplace_back(std::move(obstacle_cloud_segment));
    }

    combineSegmentedPoints(ground_cloud_segments, SegmentationLabels::GROUND, ground_cloud);
    combineSegmentedPoints(obstacle_cloud_segments, SegmentationLabels::OBSTACLE, obstacle_cloud);
}

} // namespace lidar_processing

#endif // GROUND_SEGMENTATION