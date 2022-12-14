#pragma once
#ifndef GROUND_SEGMENTATION_HPP_
#define GROUND_SEGMENTATION_HPP_

// new point types
#include "point_types.hpp"

// std
#include <algorithm>
#include <array>
#include <memory>
#include <numeric>
#include <random>
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
struct GroundPlane
{
    float a, b, c, d;
    GroundPlane(float _a, float _b, float _c, float _d) : a(_a), b(_b), c(_c), d(_d){};
    ~GroundPlane() = default;
};

class GroundSegmentation
{
  public:
    GroundSegmentation(unsigned int number_of_iterations = 3U, unsigned int number_of_segments = 2U,
                       unsigned int number_of_lowest_point_representative_estimators = 400U,
                       float sensor_height = 1.73F, float distance_threshold = 0.3F,
                       float initial_seed_threshold = 0.6F)
        : number_of_iterations_(number_of_iterations), number_of_segments_(number_of_segments),
          number_of_lowest_point_representative_estimators_(number_of_lowest_point_representative_estimators),
          sensor_height_(sensor_height), distance_threshold_(distance_threshold),
          initial_seed_threshold_(initial_seed_threshold)
    {
    }

    ~GroundSegmentation() = default;

    void segmentGround(const pcl::PointCloud<pcl::PointXYZI> &input_cloud,
                       pcl::PointCloud<pcl::PointXYZRGBI> &ground_cloud,
                       pcl::PointCloud<pcl::PointXYZRGBI> &obstacle_cloud);

  private:
    float sensor_height_;
    float distance_threshold_;
    float initial_seed_threshold_;

    unsigned int number_of_iterations_;
    unsigned int number_of_segments_;
    unsigned int number_of_lowest_point_representative_estimators_;

    void formSegments(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &cloud_segments);

    template <typename PointT>
    void extractInitialSeeds(const typename pcl::PointCloud<PointT> &cloud, std::vector<int> &indices);

    GroundPlane estimatePlane(const Eigen::MatrixXf &points_xyz);

    void fitGroundPlane(const pcl::PointCloud<pcl::PointXYZI> &cloud_segment,
                        pcl::PointCloud<pcl::PointXYZI> &ground_cloud_segment,
                        pcl::PointCloud<pcl::PointXYZI> &obstacle_cloud_segment);
};
} // namespace lidar_processing

#endif // GROUND_SEGMENTATION_HPP_