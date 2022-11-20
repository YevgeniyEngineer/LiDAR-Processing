#pragma once
#ifndef CONVERSION_HPP_
#define CONVERSION_HPP_

// std
#include <chrono>
#include <cstring>
#include <memory>

// ROS2
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
// convert pcl::PointCloud<PointT> to pcl::PCLPointCloud2
template <typename PointT>
void convert(const typename pcl::PointCloud<PointT> &pcl_cloud, pcl::PCLPointCloud2 &pcl_message)
{
    pcl::toPCLPointCloud2<PointT>(pcl_cloud, pcl_message);
}

// convert pcl::PCLPointCloud2 to sensor_msgs::msg::PointCloud2
void convert(const pcl::PCLPointCloud2 &pcl_message, sensor_msgs::msg::PointCloud2 &ros2_message)
{
    for (const auto &pcl_field : pcl_message.fields)
    {
        sensor_msgs::msg::PointField ros2_field;
        ros2_field.name = pcl_field.name;
        ros2_field.offset = pcl_field.offset;
        ros2_field.datatype = pcl_field.datatype;
        ros2_field.count = pcl_field.count;
        ros2_message.fields.emplace_back(ros2_field);
    }

    ros2_message.height = pcl_message.height;
    ros2_message.width = pcl_message.width;
    ros2_message.is_bigendian = pcl_message.is_bigendian;
    ros2_message.point_step = pcl_message.point_step;
    ros2_message.row_step = pcl_message.row_step;
    ros2_message.data = pcl_message.data;
    ros2_message.is_dense = pcl_message.is_dense;
}

// convert pcl::PointCloud<PointT> to sensor_msgs::msg::PointCloud2
template <typename PointT>
void convert(const typename pcl::PointCloud<PointT> &pcl_cloud, sensor_msgs::msg::PointCloud2 &ros2_message)
{
    pcl::PCLPointCloud2::Ptr pcl_message = std::make_shared<pcl::PCLPointCloud2>();
    convert<PointT>(pcl_cloud, *pcl_message);
    convert(*pcl_message, ros2_message);
}

// convert sensor_msgs::msg::PointCloud2 to pcl::PCLPointCloud2
void convert(const sensor_msgs::msg::PointCloud2 &ros2_message, pcl::PCLPointCloud2 &pcl_message)
{
    for (const auto &ros2_field : ros2_message.fields)
    {
        pcl::PCLPointField pcl_field;
        pcl_field.name = ros2_field.name;
        pcl_field.offset = ros2_field.offset;
        pcl_field.datatype = ros2_field.datatype;
        pcl_field.count = ros2_field.count;
        pcl_message.fields.emplace_back(pcl_field);
    }

    pcl_message.height = ros2_message.height;
    pcl_message.width = ros2_message.width;
    pcl_message.is_bigendian = ros2_message.is_bigendian;
    pcl_message.point_step = ros2_message.point_step;
    pcl_message.row_step = ros2_message.row_step;
    pcl_message.data = ros2_message.data;
    pcl_message.is_dense = ros2_message.is_dense;
}

// convert pcl::PCLPointCloud2 to pcl::PointCloud<PointT>
template <typename PointT>
void convert(const pcl::PCLPointCloud2 &pcl_message, typename pcl::PointCloud<PointT> &pcl_cloud)
{
    pcl::fromPCLPointCloud2<PointT>(pcl_message, pcl_cloud);
}

// convert sensor_msgs::msg::PointCloud2 to pcl::PointCloud<PointT>
template <typename PointT>
void convert(const sensor_msgs::msg::PointCloud2 &ros2_message, typename pcl::PointCloud<PointT> &pcl_cloud)
{
    pcl::PCLPointCloud2::Ptr pcl_message = std::make_shared<pcl::PCLPointCloud2>();
    convert(ros2_message, *pcl_message);
    convert<PointT>(*pcl_message, pcl_cloud);
}

} // namespace lidar_processing

#endif // CONVERSION_HPP_