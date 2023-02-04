#ifndef GROUND_SEGMENTATION
#define GROUND_SEGMENTATION

// STL
#include <algorithm>
#include <cstdint>
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
          number_of_lower_point_representative_estimators_(number_of_lowest_point_representative_estimators),
          sensor_height_(sensor_height), distance_threshold_(distance_threshold),
          initial_seed_threshold_(initial_seed_threshold){};

    virtual ~GroundSegmenter() = default;

    template <typename PointT>
    void segmentGround(const pcl::PointCloud<PointT> &input_cloud, pcl::PointCloud<pcl::PointXYZRGB> &ground_cloud,
                       pcl::PointCloud<pcl::PointXYZRGBL> &obstacle_cloud);

  private:
    template <typename PointT>
    void formSegments(const pcl::PointCloud<PointT> &cloud, std::vector<pcl::PointCloud<PointT>::Ptr> &cloud_segments);

    template <typename PointT>
    void extractInitialSeeds(const pcl::PointCloud<PointT> &cloud, std::vector<int> &indices);

    template <typename PointT>
    void fitGroundPlane(const pcl::PointCloud<PointT> &cloud_segment,
                        pcl::PointCloud<pcl::PointXYZRGBL> &ground_cloud_segment,
                        pcl::PointCloud<pcl::PointXYZRGBL> &obstacle_cloud_segment);

  private:
    const std::uint32_t number_of_iterations_;
    const std::uint32_t number_of_planar_partitions_;
    const std::uint32_t number_of_lower_point_representative_estimators_;
    const float sensor_height_;
    const float distance_threshold_;
    const float initial_seed_threshold_;
};
} // namespace lidar_processing

#endif // GROUND_SEGMENTATION