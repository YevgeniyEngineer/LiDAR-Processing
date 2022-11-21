#pragma once
#ifndef GROUND_SEGMENTATION_NODE_HPP_
#define GROUND_SEGMENTATION_NODE_HPP_

// custom point types
#include "conversion.hpp"
#include "point_types.hpp"

// ground segmentation
#include "ground_segmentation.hpp"

// std
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// ROS2
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/for_each_type.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
// Segments ground points from point cloud coming from data reader node
class GroundSegmentationNode : public rclcpp::Node
{
  public:
    // constructor
    GroundSegmentationNode();

    // destructor
    ~GroundSegmentationNode() = default;

  private:
    // subscription message
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

    // publication message
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    // segmentation callback function
    void segmentGround(const sensor_msgs::msg::PointCloud2 &input_message);
};
} // namespace lidar_processing

#endif // GROUND_SEGMENTATION_NODE_HPP_segmented_pointcloud