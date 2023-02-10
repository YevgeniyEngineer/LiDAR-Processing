// Processing
#include "conversions.hpp"
#include "convex_hull.hpp"
#include "ground_segmentation.hpp"
#include "obstacle_clustering.hpp"

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
    std::unique_ptr<GroundSegmenter> ground_segmenter = std::make_unique<GroundSegmenter>();
    std::unique_ptr<ObstacleClusterer> obstacle_clusterer = std::make_unique<ObstacleClusterer>();

    // Convert PointCloud2 to pcl::PointCloud<pcl::PointXYZI>
    std::unique_ptr<pcl::PointCloud<pcl::PointXYZI>> point_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
    convertPointCloud2ToPCL(input_message, *point_cloud);

    // Ground segmentation
    const auto &ground_segmentation_start_time = std::chrono::high_resolution_clock::now();

    std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGBL>> ground_cloud =
        std::make_unique<pcl::PointCloud<pcl::PointXYZRGBL>>();
    std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGBL>> obstacle_cloud =
        std::make_unique<pcl::PointCloud<pcl::PointXYZRGBL>>();
    ground_segmenter->segmentGround(*point_cloud, *ground_cloud, *obstacle_cloud);

    const auto &ground_segmentation_end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Ground segmentation time: "
              << (ground_segmentation_end_time - ground_segmentation_start_time).count() / 1e9 << std::endl;

    // std::cout << "Ground Pts: " << ground_cloud->size() << " | Obstacle Pts: " << obstacle_cloud->size() <<
    // std::endl;

    // Obstacle clustering
    const auto &obstacle_clustering_start_time = std::chrono::high_resolution_clock::now();

    std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGBL>> clustered_obstacle_cloud =
        std::make_unique<pcl::PointCloud<pcl::PointXYZRGBL>>();
    obstacle_clusterer->clusterObstacles(*obstacle_cloud, *clustered_obstacle_cloud);

    const auto &obstacle_clustering_end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Obstacle clustering time: "
              << (obstacle_clustering_end_time - obstacle_clustering_start_time).count() / 1e9 << std::endl;
    // std::cout << "Number of clusters: " << clustered_obstacle_cloud.size() << std::endl;

    // Polygonization
    const auto &convex_polygon_simplification_start_time = std::chrono::high_resolution_clock::now();

    std::vector<Point<float>> clustered_obstacle_points;
    clustered_obstacle_points.reserve(clustered_obstacle_cloud->size());
    for (int point_no = 0; point_no < clustered_obstacle_cloud->size(); ++point_no)
    {
        const auto &point = clustered_obstacle_cloud->points[point_no];
        clustered_obstacle_points.emplace_back(point.x, point.y, point_no);
    }
    ConvexHullGenerator<float> convex_hull_generator{clustered_obstacle_points, true};
    const std::vector<Point<float>> &convex_hull_points = convex_hull_generator.getConvexHull();
    // std::cout << "Number of convex hull points: " << convex_hull_points.size() << std::endl;

    const auto &convex_polygon_simplification_end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Convex polygon simplification time: "
              << (convex_polygon_simplification_end_time - convex_polygon_simplification_start_time).count() / 1e9
              << std::endl;

    // ***** Publish data for visualisation *****
    // Convert to ROS2 format and publish (ground segmentation)
    if (!ground_cloud->empty())
    {
        std::unique_ptr<PointCloud2> output_ground_segmentation_message = std::make_unique<PointCloud2>();
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
        std::unique_ptr<PointCloud2> output_obstacle_segmentation_message = std::make_unique<PointCloud2>();
        output_obstacle_segmentation_message->header = input_message.header;
        output_obstacle_segmentation_message->is_bigendian = input_message.is_bigendian;
        convertPCLToPointCloud2(*obstacle_cloud, *output_obstacle_segmentation_message);
        publisher_obstacle_cloud_->publish(*output_obstacle_segmentation_message);
    }
    else
    {
        std::cout << "No obstacle points!" << std::endl;
    }

    // Convert to ROS2 format and publish (obstacle clustering - polygonization)
}
} // namespace lidar_processing

int main(int argc, const char **argv)
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