#pragma once
#ifndef OBSTACLE_CLUSTERING_NODE_HPP_
#define OBSTACLE_CLUSTERING_NODE_HPP_

// custom point types
#include "conversion.hpp"
#include "point_types.hpp"

// obstacle clustering
#include "obstacle_clustering.hpp"

// std
#include <chrono>
#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// ROS2
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/for_each_type.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
// Applies Euclidean clustering on obstacles
class ObstacleClusteringNode : public rclcpp::Node
{
  public:
    // constructor
    ObstacleClusteringNode();

    // destructor
    ~ObstacleClusteringNode() = default;

  private:
    // subscription message
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

    // publication message
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_marker_array_;

    // clustering callback function
    void clusterObstacles(const sensor_msgs::msg::PointCloud2 &input_message);
};
} // namespace lidar_processing

#endif // OBSTACLE_CLUSTERING_NODE_HPP_