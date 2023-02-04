// Processing
#include "conversions.hpp"
#include "ground_segmentation.hpp"

// STL
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

// ROS2
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
class ProcessingNode : public rclcpp::Node
{
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using PointFieldTypes = pcl::PCLPointField::PointFieldTypes;

  public:
    ProcessingNode() : rclcpp::Node::Node("processing_node")
    {
        std::cout << "processing_node started" << std::endl;

        // Subscriber will receive messages from data_reader_node
        subscriber_ = this->create_subscription<PointCloud2>(
            "pointcloud", 10, std::bind(&ProcessingNode::process, this, std::placeholders::_1));

        // Publisher nodes
        publisher_ground_cloud_ = this->create_publisher<PointCloud2>("ground_pointcloud", 10);
        publisher_obstacle_cloud_ = this->create_publisher<PointCloud2>("obstacle_pointcloud", 10);
    }

    ~ProcessingNode() = default;

    void process(const PointCloud2 &input_message);

  private:
    // Subscriber receives raw point cloud data
    rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;

    // Ground segmentation
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_ground_cloud_;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_obstacle_cloud_;

    // Obstacle clustering
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_obstacle_clustering_cloud_;

    // Obstacle cluster polygonization
    rclcpp::Publisher<MarkerArray>::SharedPtr publisher_obstacle_convex_hulls_;
    rclcpp::Publisher<MarkerArray>::SharedPtr publisher_obstacle_concave_hulls_;
};

void ProcessingNode::process(const PointCloud2 &input_message)
{
    // Instantiate processing objects
    GroundSegmenter ground_segmenter;

    // Convert PointCloud2 to pcl::PointCloud<pcl::PointXYZI>
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
    convertPointCloud2ToPCL(input_message, point_cloud);

    // Ground segmentation
    pcl::PointCloud<pcl::PointXYZRGBL> ground_cloud;
    pcl::PointCloud<pcl::PointXYZRGBL> obstacle_cloud;
    ground_segmenter.segmentGround(point_cloud, ground_cloud, obstacle_cloud);

    // Convert to ROS2 format
    PointCloud2 output_ground_segmentation_message;
    PointCloud2 output_obstacle_segmentation_message;

    if (!ground_cloud.empty())
    {
        output_ground_segmentation_message.header = input_message.header;
        output_ground_segmentation_message.is_bigendian = input_message.is_bigendian;
        convertPCLToPointCloud2(ground_cloud, output_ground_segmentation_message);
        publisher_ground_cloud_->publish(output_ground_segmentation_message);
    }
    if (!obstacle_cloud.empty())
    {
        output_obstacle_segmentation_message.header = input_message.header;
        output_obstacle_segmentation_message.is_bigendian = input_message.is_bigendian;
        convertPCLToPointCloud2(obstacle_cloud, output_obstacle_segmentation_message);
        publisher_obstacle_cloud_->publish(output_obstacle_segmentation_message);
    }
}
} // namespace lidar_processing

int main(const int argc, const char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

    bool success = true;
    try
    {
        rclcpp::spin(std::make_shared<lidar_processing::ProcessingNode>());
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