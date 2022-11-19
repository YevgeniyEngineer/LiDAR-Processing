#pragma once
#ifndef DATA_READER_NODE_HPP_
#define DATA_READER_NODE_HPP_

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

// Data path
std::string filepath = "/home/yevgeniy/Documents/GitHub/LiDAR-Processing/data";

namespace lidar_processing
{
// Reads pcd data on repeat at 10Hz
class PointCloudPublisher : public rclcpp::Node
{
  public:
    // constructor
    PointCloudPublisher();

    // destructor
    ~PointCloudPublisher() = default;

  private:
    // timer callback function used by the timer
    void timerCallback();

    // function that reads file names
    std::vector<std::filesystem::path> readFilenamesExt(std::filesystem::path const & /* path */,
                                                        std::string const & /* extension */);

    // message published by this node
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    // timer
    std::shared_ptr<rclcpp::TimerBase> timer_;

    // for file reading and iteration
    std::string filename_;
    std::vector<std::filesystem::path> filenames_;
    std::vector<std::filesystem::path>::iterator filenames_iterator_;
};
} // namespace lidar_processing

#endif // DATA_READER_NODE_HPP_