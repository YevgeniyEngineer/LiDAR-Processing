#pragma once
#ifndef GROUND_SEGMENTATION_HPP_
#define GROUND_SEGMENTATION_HPP_

#include "point_types.hpp"
#include <eigen3/Eigen/Dense>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
struct GroundPlane
{
    float a, b, c, d;
    GroundPlane(float _a, float _b, float _c, float _d) : a(_a), b(_b), c(_c), d(_d){};
    ~GroundPlane() = default;
};

class GroundSegmentation
{
  public:
    GroundSegmentation(unsigned int number_of_iterations = 3U, unsigned int number_of_segments = 1U,
                       unsigned int number_of_lowest_point_representative_estimators = 250U,
                       float sensor_height = 1.73F, float distance_threshold = 0.3F,
                       float initial_seed_threshold = 1.2F);
    ~GroundSegmentation() = default;

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

    GroundPlane estimatePlane(const pcl::PointCloud<pcl::PointXYZI> &ground_cloud);
    void extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI> &cloud);
};
} // namespace lidar_processing

#endif // GROUND_SEGMENTATION_HPP_