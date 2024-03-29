#ifndef CONVERSIONS
#define CONVERSIONS

// Point<T>
#include "internal_types.hpp"

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