#ifndef CONVERSIONS
#define CONVERSIONS

// Point<T>
#include "internal_types.hpp"

// STL
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// ROS2 Message Types
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
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

void convertPointCloud2ToPCL(const sensor_msgs::msg::PointCloud2 &cloud_ros, pcl::PointCloud<pcl::PointXYZI> &cloud_pcl)
{
    cloud_pcl.width = cloud_ros.width;
    cloud_pcl.height = cloud_ros.height;
    cloud_pcl.is_dense = cloud_ros.is_dense;

    cloud_pcl.points.clear();
    cloud_pcl.points.resize(cloud_pcl.width * cloud_pcl.height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_ros, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_ros, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_ros, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(cloud_ros, "intensity");

    int i = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++i)
    {
        cloud_pcl.points[i].x = *iter_x;
        cloud_pcl.points[i].y = *iter_y;
        cloud_pcl.points[i].z = *iter_z;
        cloud_pcl.points[i].intensity = *iter_intensity;
    }
}

// Does not set header and endianess!
void convertPCLToPointCloud2(const pcl::PointCloud<pcl::PointXYZRGBL> &cloud_pcl,
                             sensor_msgs::msg::PointCloud2 &cloud_ros)
{
    cloud_ros.is_dense = cloud_pcl.is_dense;
    cloud_ros.height = cloud_pcl.height;
    cloud_ros.width = cloud_pcl.width;
    cloud_ros.point_step = sizeof(pcl::PointXYZRGBL);

    std::vector<std::tuple<std::string, std::uint32_t, std::uint8_t, std::uint32_t>> fields = {
        {"x", offsetof(pcl::PointXYZRGBL, x), PointFieldTypes::FLOAT32, 1},
        {"y", offsetof(pcl::PointXYZRGBL, y), PointFieldTypes::FLOAT32, 1},
        {"z", offsetof(pcl::PointXYZRGBL, z), PointFieldTypes::FLOAT32, 1},
        {"rgb", offsetof(pcl::PointXYZRGBL, rgb), PointFieldTypes::FLOAT32, 1},
        {"label", offsetof(pcl::PointXYZRGBL, label), PointFieldTypes::UINT32, 1}};

    for (const auto &field : fields)
    {
        sensor_msgs::msg::PointField field_cache;
        field_cache.name = std::get<0>(field);
        field_cache.offset = std::get<1>(field);
        field_cache.datatype = std::get<2>(field);
        field_cache.count = std::get<3>(field);
        cloud_ros.fields.emplace_back(std::move(field_cache));
    }

    const std::size_t byte_size = sizeof(pcl::PointXYZRGBL) * cloud_pcl.size();
    cloud_ros.data.clear();
    cloud_ros.data.resize(byte_size);
    std::memcpy(cloud_ros.data.data(), &cloud_pcl.at(0), byte_size);
}

// Convert std::*** to MarkerArray
void convertPointXYZTypeToMarkerArray(const std::vector<std::vector<Point<float>>> &hull_cluster_points,
                                      const std::string &frame_id, const builtin_interfaces::msg::Time &stamp,
                                      visualization_msgs::msg::MarkerArray &marker_array)
{
    marker_array.markers.clear();
    marker_array.markers.reserve(hull_cluster_points.size());
    for (int hull_no = 0; hull_no < hull_cluster_points.size(); ++hull_no)
    {
        const auto &hull_points = hull_cluster_points[hull_no];

        visualization_msgs::msg::Marker marker;

        marker.lifetime.sec = 0;
        marker.lifetime.nanosec = 100'000'000;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = "convex_hull";
        marker.id = hull_no;

        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.15;
        marker.color.a = 1.0f;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;

        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.points.reserve(hull_points.size());
        for (const auto &point : hull_points)
        {
            geometry_msgs::msg::Point point_cache;
            point_cache.x = static_cast<double>(point.x);
            point_cache.y = static_cast<double>(point.y);
            point_cache.z = 0.0;
            marker.points.emplace_back(std::move(point_cache));
        }

        marker_array.markers.emplace_back(std::move(marker));
    }
}

} // namespace lidar_processing

#endif // CONVERSIONS