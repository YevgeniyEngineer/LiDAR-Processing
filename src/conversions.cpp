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

#include "conversions.hpp"
#include <cstddef>
#include <cstdint>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/detail/point_field__struct.hpp>

namespace lidar_processing
{
pcl::PointCloud<pcl::PointXYZRGB> convertClusteredCloudToColorizedCloud(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>> &clustered_cloud)
{
    // Calculate space required for preallocation
    std::size_t reservation_size = 0;
    for (const auto &cluster : clustered_cloud)
    {
        reservation_size += cluster.size();
    }
    pcl::PointCloud<pcl::PointXYZRGB> colorized_cloud{};
    colorized_cloud.reserve(reservation_size);

    // Iterate through each cluster in the input vector
    for (const auto &cluster : clustered_cloud)
    {
        // Generate random RGB values for the current cluster
        auto r = static_cast<std::uint8_t>(std::rand() % 256);
        auto g = static_cast<std::uint8_t>(std::rand() % 256);
        auto b = static_cast<std::uint8_t>(std::rand() % 256);

        // Iterate through each point in the current cluster
        for (const auto &point : cluster)
        {
            colorized_cloud.push_back(pcl::PointXYZRGB(point.x, point.y, point.z, r, g, b));
        }
    }

    return colorized_cloud;
}

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
        auto &point = cloud_pcl.points[i];
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        point.intensity = *iter_intensity;
    }
}

void convertPCLToPointCloud2(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_pcl,
                             sensor_msgs::msg::PointCloud2 &cloud_ros)
{
    cloud_ros.is_dense = cloud_pcl.is_dense;
    cloud_ros.height = cloud_pcl.height;
    cloud_ros.width = cloud_pcl.width;
    cloud_ros.point_step = sizeof(pcl::PointXYZRGB);

    std::vector<std::tuple<std::string, std::uint32_t, std::uint8_t, std::uint32_t>> fields = {
        {"x", offsetof(pcl::PointXYZRGB, x), PointFieldTypes::FLOAT32, 1},
        {"y", offsetof(pcl::PointXYZRGB, y), PointFieldTypes::FLOAT32, 1},
        {"z", offsetof(pcl::PointXYZRGB, z), PointFieldTypes::FLOAT32, 1},
        {"rgb", offsetof(pcl::PointXYZRGB, rgb), PointFieldTypes::FLOAT32, 1}};

    for (const auto &field : fields)
    {
        sensor_msgs::msg::PointField field_cache;
        field_cache.name = std::get<0>(field);
        field_cache.offset = std::get<1>(field);
        field_cache.datatype = std::get<2>(field);
        field_cache.count = std::get<3>(field);
        cloud_ros.fields.emplace_back(std::move(field_cache));
    }

    const std::size_t byte_size = sizeof(pcl::PointXYZRGB) * cloud_pcl.size();
    cloud_ros.data.clear();
    cloud_ros.data.resize(byte_size);
    std::memcpy(cloud_ros.data.data(), &cloud_pcl.at(0), byte_size);
}

void convertPCLToPointCloud2(const pcl::PointCloud<pcl::PointXYZ> &cloud_pcl, sensor_msgs::msg::PointCloud2 &cloud_ros,
                             std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
    cloud_ros.is_dense = cloud_pcl.is_dense;
    cloud_ros.height = 1;
    cloud_ros.width = cloud_pcl.points.size();
    cloud_ros.point_step = sizeof(pcl::PointXYZRGB);

    std::vector<std::tuple<std::string, std::uint32_t, std::uint8_t, std::uint32_t>> fields = {
        {"x", offsetof(pcl::PointXYZRGB, x), PointFieldTypes::FLOAT32, 1},
        {"y", offsetof(pcl::PointXYZRGB, y), PointFieldTypes::FLOAT32, 1},
        {"z", offsetof(pcl::PointXYZRGB, z), PointFieldTypes::FLOAT32, 1},
        {"rgb", offsetof(pcl::PointXYZRGB, rgb), PointFieldTypes::FLOAT32, 1}};

    sensor_msgs::msg::PointField field_cache;
    for (const auto &field : fields)
    {
        field_cache.name = std::get<0>(field);
        field_cache.offset = std::get<1>(field);
        field_cache.datatype = std::get<2>(field);
        field_cache.count = std::get<3>(field);
        cloud_ros.fields.push_back(field_cache);
    }

    const std::size_t byte_size = sizeof(pcl::PointXYZRGB) * cloud_pcl.size();
    cloud_ros.data.clear();
    cloud_ros.data.resize(byte_size);

    pcl::PointXYZRGB point_cache;
    point_cache.r = r;
    point_cache.g = g;
    point_cache.b = b;

    std::ptrdiff_t pointer_offset = 0;
    for (const auto &point : cloud_pcl.points)
    {
        point_cache.x = point.x;
        point_cache.y = point.y;
        point_cache.z = point.z;

        std::memcpy(static_cast<void *>(&cloud_ros.data[pointer_offset]), static_cast<const void *>(&point_cache),
                    sizeof(pcl::PointXYZRGB));

        pointer_offset += sizeof(pcl::PointXYZRGB);
    }
}

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

} // namespace lidar_processing
