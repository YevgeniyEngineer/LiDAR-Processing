#pragma once
#ifndef TRANSFORM_NODE_HPP_
#define TRANSFORM_NODE_HPP_

// std
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// ROS2
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster_node.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace lidar_processing
{
class PosePublisher : public rclcpp::Node
{
  public:
    // default constructor - sets all values to 0.0F
    PosePublisher();

    // constructor
    PosePublisher(float tx, float ty, float tz, float rx, float ry, float rz);

    // destructor
    ~PosePublisher() = default;

  private:
    // pose callback function used by static_transform_stamped_
    void poseCallback();

    // timer
    std::shared_ptr<rclcpp::TimerBase> timer_;

    // message published by this node
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;

    // static transform broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    // static transforms
    float tx_, ty_, tz_;
    float rx_, ry_, rz_;
};
} // namespace lidar_processing

#endif // TRANSFORM_NODE_HPP_