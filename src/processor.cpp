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

// External
#include "convex_hull.hpp"

// Processing
#include "clustering.hpp"
#include "conversions.hpp"
#include "polygon_simplification.hpp"
#include "segmentation.hpp"

// STL
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// ROS2
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PCL
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
class Processor : public rclcpp::Node
{
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using PointFieldTypes = pcl::PCLPointField::PointFieldTypes;

  public:
    Processor() : rclcpp::Node::Node("processor")
    {
        std::cout << "processing_node started" << std::endl;

        // Subscriber will receive messages from data_reader_node
        rclcpp::QoS qos(2);
        qos.keep_last(2);
        qos.reliable();
        qos.durability_volatile();
        qos.liveliness(rclcpp::LivelinessPolicy::SystemDefault);

        // How long a node must wait before declaring itself "alive" to the rest
        // of the system again If the node fails to send out a liveliness
        // message within the specified lease duration, it is considered "dead"
        // or "unresponsive" by the rest of the system
        qos.liveliness_lease_duration(std::chrono::seconds(1));

        // How long a node must wait for a response from a remote node before
        // declaring it as "dead" or "unresponsive" If the remote node fails to
        // respond within the specified deadline, the requesting node considers
        // the remote node as "dead" or "unresponsive"
        qos.deadline(std::chrono::seconds(1));

        // Preallocate memory
        cloud_in_.reserve(200'000);
        ground_points_.reserve(200'000);
        obstacle_points_.reserve(200'000);

        // Create publishers and subscribers
        subscriber_ = this->create_subscription<PointCloud2>(
            "pointcloud", qos, std::bind(&Processor::process, this, std::placeholders::_1));

        // Publisher nodes
        publisher_ground_cloud_ = this->create_publisher<PointCloud2>("ground_pointcloud", qos);
        publisher_obstacle_cloud_ = this->create_publisher<PointCloud2>("obstacle_pointcloud", qos);
        publisher_clustered_cloud_ = this->create_publisher<PointCloud2>("clustered_pointcloud", qos);
        publisher_polygonization_ = this->create_publisher<MarkerArray>("polygonization", qos);
    }

    ~Processor() = default;

    void process(const PointCloud2 &input_message);

  private:
    // Subscriber receives raw point cloud data
    rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;

    // Ground segmentation
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_ground_cloud_;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_obstacle_cloud_;

    // Obstacle clustering
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_clustered_cloud_;

    // Obstacle cluster polygonization
    rclcpp::Publisher<MarkerArray>::SharedPtr publisher_polygonization_;
    rclcpp::Publisher<MarkerArray>::SharedPtr publisher_obstacle_concave_hulls_;

    // Cache
    pcl::PointCloud<pcl::PointXYZI> cloud_in_;
    pcl::PointCloud<pcl::PointXYZI> ground_points_;
    pcl::PointCloud<pcl::PointXYZI> obstacle_points_;
    std::vector<SegmentationLabel> segmentation_labels_;

    // Segmenter
    Segmenter segmenter_;

    // Clusterer
    Clusterer clusterer_;
};

void Processor::process(const PointCloud2 &input_message)
{
    convertPointCloud2ToPCL(input_message, cloud_in_);

    // auto downsampled_point_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
    // downsampled_point_cloud->points.reserve(point_cloud->points.size());

    // pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_downsampler;
    // voxel_grid_downsampler.setInputCloud(cloud_in_);
    // voxel_grid_downsampler.setLeafSize(0.06f, 0.06f, 0.06f);
    // voxel_grid_downsampler.filter(*downsampled_point_cloud);

    // Ground segmentation
    const auto ground_segmentation_start_time = std::chrono::high_resolution_clock::now();

    segmenter_.segment(cloud_in_, segmentation_labels_, ground_points_, obstacle_points_);

    auto ground_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZRGBL>>();
    ground_cloud->reserve(ground_points_.size());
    for (const auto &ground_point : ground_points_)
    {
        ground_cloud->emplace_back(ground_point.x, ground_point.y, ground_point.z, 220, 220, 220, 0);
    }
    auto obstacle_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZRGBL>>();
    obstacle_cloud->reserve(obstacle_points_.size());
    for (const auto &obstacle_point : obstacle_points_)
    {
        obstacle_cloud->emplace_back(obstacle_point.x, obstacle_point.y, obstacle_point.z, 0, 255, 0, 1);
    }

    const auto ground_segmentation_end_time = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO(this->get_logger(), "Ground segmentation time [ms]: %lf",
                (ground_segmentation_end_time - ground_segmentation_start_time).count() / 1.0e6);

    RCLCPP_INFO(this->get_logger(), "Ground Pts: %lu | Obstacle Pts: %lu", ground_cloud->size(),
                obstacle_points_.size());

    // Obstacle clustering
    const auto obstacle_clustering_start_time = std::chrono::high_resolution_clock::now();

    std::vector<pcl::PointCloud<pcl::PointXYZ>> clustered_obstacle_cloud;
    std::vector<ClusteringLabel> cluster_labels;
    clusterer_.cluster(*obstacle_cloud, cluster_labels);

    const auto max_label = *std::max_element(cluster_labels.cbegin(), cluster_labels.cend());
    clustered_obstacle_cloud.resize(max_label + 1);

    for (std::size_t i = 0; i < obstacle_cloud->size(); ++i)
    {
        auto label = cluster_labels[i];
        if (label == Clusterer::UNDEFINED)
        {
            throw std::runtime_error("Undefined label found (clustering)");
        }
        if (label != Clusterer::INVALID)
        {
            const auto &point = obstacle_cloud->points[i];
            clustered_obstacle_cloud[label].emplace_back(point.x, point.y, point.z);
        }
    }

    clustered_obstacle_cloud.erase(
        std::remove_if(clustered_obstacle_cloud.begin(), clustered_obstacle_cloud.end(),
                       [](const pcl::PointCloud<pcl::PointXYZ> &cloud) -> bool { return cloud.empty(); }),
        clustered_obstacle_cloud.end());

    const auto obstacle_clustering_end_time = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO(this->get_logger(), "Obstacle clustering time [ms]: %lf",
                (obstacle_clustering_end_time - obstacle_clustering_start_time).count() / 1.0e6);

    RCLCPP_INFO(this->get_logger(), "Number of clusters: %lu", clustered_obstacle_cloud.size());

    // Polygonization
    const auto polygonization_start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<geom::Point<float>>> cluster_outlines;
    // findOrderedConvexOutlines(clustered_obstacle_cloud, cluster_outlines);
    findOrderedConcaveOutlines(clustered_obstacle_cloud, cluster_outlines);

    const auto polygonization_end_time = std::chrono::high_resolution_clock::now();

    RCLCPP_INFO(this->get_logger(), "Polygonization time [ms]: %lf",
                (polygonization_end_time - polygonization_start_time).count() / 1.0e6);

    // ***** Publish data for visualisation *****
    // Convert to ROS2 format and publish (ground segmentation)
    if (!ground_cloud->empty())
    {
        auto output_ground_segmentation_message = std::make_unique<PointCloud2>();
        output_ground_segmentation_message->header = input_message.header;
        output_ground_segmentation_message->is_bigendian = input_message.is_bigendian;
        convertPCLToPointCloud2(*ground_cloud, *output_ground_segmentation_message);
        publisher_ground_cloud_->publish(*output_ground_segmentation_message);
    }
    else
    {
        std::cout << "No ground points!" << std::endl;
    }

    if (!obstacle_cloud->empty())
    {
        auto output_obstacle_segmentation_message = std::make_unique<PointCloud2>();
        output_obstacle_segmentation_message->header = input_message.header;
        output_obstacle_segmentation_message->is_bigendian = input_message.is_bigendian;
        convertPCLToPointCloud2(*obstacle_cloud, *output_obstacle_segmentation_message);
        publisher_obstacle_cloud_->publish(*output_obstacle_segmentation_message);
    }
    else
    {
        std::cout << "No obstacle points!" << std::endl;
    }

    // Convert to ROS2 format and publish (clustering)
    if (!clustered_obstacle_cloud.empty())
    {
        auto output_clustered_obstacle_message = std::make_unique<PointCloud2>();
        output_clustered_obstacle_message->header = input_message.header;
        output_clustered_obstacle_message->is_bigendian = input_message.is_bigendian;
        auto colorized_cloud = convertClusteredCloudToColorizedCloud(clustered_obstacle_cloud);
        convertPCLToPointCloud2(colorized_cloud, *output_clustered_obstacle_message);
        publisher_clustered_cloud_->publish(*output_clustered_obstacle_message);
    }

    // Convert to ROS2 format and publish (polygonization)
    if (!cluster_outlines.empty())
    {
        auto output_convex_polygonization_message = std::make_unique<MarkerArray>();
        convertPointXYZTypeToMarkerArray(cluster_outlines, input_message.header.frame_id, input_message.header.stamp,
                                         *output_convex_polygonization_message);
        publisher_polygonization_->publish(*output_convex_polygonization_message);
    }
}
} // namespace lidar_processing

int main(int argc, const char **const argv)
{
    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

    bool success = true;
    try
    {
        rclcpp::spin(std::make_shared<lidar_processing::Processor>());
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
        success = false;
    }
    catch (...)
    {
        std::cerr << "Unknown exception!" << std::endl;
        success = false;
    }

    rclcpp::shutdown();

    if (!success)
    {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
