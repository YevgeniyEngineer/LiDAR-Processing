/*
 * Copyright (c) 2024 Yevgeniy Simonov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef CONVERSIONS
#define CONVERSIONS

// STL
#include <cstdint>
#include <cstdlib> // for std::rand()
#include <cstring>
#include <string>
#include <vector>

// ROS2 Message Types
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using PointFieldTypes = pcl::PCLPointField::PointFieldTypes;

void convertPointCloud2ToPCL(const sensor_msgs::msg::PointCloud2 &cloud_ros,
                             pcl::PointCloud<pcl::PointXYZI> &cloud_pcl);

// Convert clouds
pcl::PointCloud<pcl::PointXYZRGB> convertClusteredCloudToColorizedCloud(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>> &clustered_cloud);

// Convert from pcl::PointCloud<pcl::PointXYZRGB> to PointCloud2
void convertPCLToPointCloud2(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_pcl,
                             sensor_msgs::msg::PointCloud2 &cloud_ros);

// Does not set header and endianess!
void convertPCLToPointCloud2(const pcl::PointCloud<pcl::PointXYZ> &cloud_pcl, sensor_msgs::msg::PointCloud2 &cloud_ros,
                             std::uint8_t r, std::uint8_t g, std::uint8_t b);

// Does not set header and endianess!
void convertPCLToPointCloud2(const pcl::PointCloud<pcl::PointXYZRGBL> &cloud_pcl,
                             sensor_msgs::msg::PointCloud2 &cloud_ros);

// Convert std::*** to MarkerArray
template <typename PointT>
void convertPointXYZTypeToMarkerArray(const std::vector<std::vector<PointT>> &hull_cluster_points,
                                      const std::string &frame_id, const builtin_interfaces::msg::Time &stamp,
                                      visualization_msgs::msg::MarkerArray &marker_array)
{
    marker_array.markers.clear();
    marker_array.markers.reserve(hull_cluster_points.size());
    for (int hull_no = 0; hull_no < hull_cluster_points.size(); ++hull_no)
    {
        const auto &hull_points = hull_cluster_points[hull_no];
        if (hull_points.empty())
        {
            continue;
        }
        visualization_msgs::msg::Marker marker;
        marker.lifetime.sec = 0;
        marker.lifetime.nanosec = 150'000'000;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = "convex_hull";
        marker.id = hull_no;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1f;
        marker.color.a = 1.0f;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        // To ensure loop closure
        marker.points.reserve(hull_points.size() + 1);
        for (const auto &point : hull_points)
        {
            geometry_msgs::msg::Point point_cache;
            point_cache.x = static_cast<double>(point.x);
            point_cache.y = static_cast<double>(point.y);
            point_cache.z = 0.0;
            marker.points.emplace_back(std::move(point_cache));
        }
        marker.points.push_back(marker.points[0]);
        marker_array.markers.emplace_back(std::move(marker));
    }
}

} // namespace lidar_processing

#endif // CONVERSIONS
