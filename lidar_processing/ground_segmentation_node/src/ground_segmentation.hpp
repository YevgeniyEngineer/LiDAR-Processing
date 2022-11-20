#pragma once
#ifndef GROUND_SEGMENTATION_HPP_
#define GROUND_SEGMENTATION_HPP_

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
class GroundSegmentation
{
  public:
    GroundSegmentation();
    ~GroundSegmentation();

    void segmentGround(const pcl::PointCloud<pcl::PointXYZI> &input_cloud,
                       pcl::PointCloud<pcl::PointXYZIL> &segmented_cloud);

  private:
    float sensor_height_;
    float distance_threshold_;
    float initial_seed_threshold_;

    unsigned int number_of_iterations_;
    unsigned int number_of_segments_;
    unsigned int number_of_lowest_point_representative_estimators_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;

    void estimatePlane(const pcl::PointCloud<pcl::PointXYZI> &ground_cloud);
    void extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI> &cloud);
};
} // namespace lidar_processing

#endif // GROUND_SEGMENTATION_HPP_