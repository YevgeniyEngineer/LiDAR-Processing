// External
#include "convex_hull.hpp"

// Processing
#include "channel_based_ground_segmenter.hpp"
#include "conversions.hpp"
#include "ground_segmentation.hpp"
#include "internal_types.hpp"
#include "obstacle_clustering.hpp"
#include "polygon_simplification.hpp"
#include "ransac_ground_segmentation.hpp"

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
#include <visualization_msgs/msg/marker_array.hpp>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// StackVector
#include "utilities/stack_vector.hpp"

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

        subscriber_ = this->create_subscription<PointCloud2>(
            "pointcloud", qos, std::bind(&ProcessingNode::process, this, std::placeholders::_1));

        // Publisher nodes
        publisher_ground_cloud_ = this->create_publisher<PointCloud2>("ground_pointcloud", qos);
        publisher_obstacle_cloud_ = this->create_publisher<PointCloud2>("obstacle_pointcloud", qos);

        publisher_obstacle_clustering_cloud_ = this->create_publisher<PointCloud2>("clustered_pointcloud", qos);

        publisher_obstacle_convex_hulls_ = this->create_publisher<MarkerArray>("convex_polygonization", qos);
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

    // Ground segmentation
    ChannelBasedGroundSegmenter channel_based_ground_segmenter_;
};

void ProcessingNode::process(const PointCloud2 &input_message)
{
    // Segmentation parameters
    SegmentationAlgorithm segmentation_algorithm = SegmentationAlgorithm::RULE_BASED_APPROACH;

    // Clustering parameters
    ClusteringAlgorithm clustering_algorithm = ClusteringAlgorithm::FAST_EUCLIDEAN_CLUSTERING;

    static constexpr float neighbour_radius_threshold = 0.3;
    static constexpr float cluster_quality = 0.5;
    static constexpr std::uint32_t min_cluster_size = 5;
    static constexpr std::uint32_t max_cluster_size = std::numeric_limits<std::uint32_t>::max();

    // Instantiate processing objects
    auto ground_segmenter = std::make_unique<GroundSegmenter>();
    auto obstacle_clusterer = std::make_unique<ObstacleClusterer>(
        neighbour_radius_threshold, cluster_quality, min_cluster_size, max_cluster_size, clustering_algorithm);

    // Convert PointCloud2 to pcl::PointCloud<pcl::PointXYZI>
    auto point_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
    convertPointCloud2ToPCL(input_message, *point_cloud);

    // Ground segmentation
    const auto &ground_segmentation_start_time = std::chrono::high_resolution_clock::now();

    auto ground_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZRGBL>>();
    auto obstacle_cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZRGBL>>();

    auto ground_cloud_2 = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    auto obstacle_cloud_2 = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();

    // RANSAC
    if (segmentation_algorithm == SegmentationAlgorithm::RANSAC)
    {
        segmentGroundRANSAC(*point_cloud, *ground_cloud, *obstacle_cloud);
    }
    // Fast Segmentation
    else if (segmentation_algorithm == SegmentationAlgorithm::ITERATIVE_PLANE_FITTING)
    {
        ground_segmenter->segmentGround(*point_cloud, *ground_cloud, *obstacle_cloud);
    }
    else if (segmentation_algorithm == SegmentationAlgorithm::RULE_BASED_APPROACH)
    {
        channel_based_ground_segmenter_.setInputCloud(point_cloud->points);
        channel_based_ground_segmenter_.segment(*ground_cloud_2, *obstacle_cloud_2);
    }
    else
    {
        throw std::runtime_error("Unknown segmentation algorithm");
    }

    const auto &ground_segmentation_end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Ground segmentation time: "
              << (ground_segmentation_end_time - ground_segmentation_start_time).count() / 1e9 << std::endl;

    std::cout << "Ground Pts: " << ground_cloud->size() << " | Obstacle Pts: " << obstacle_cloud->size() << std::endl;

    // Obstacle clustering
    const auto &obstacle_clustering_start_time = std::chrono::high_resolution_clock::now();

    std::vector<pcl::PointCloud<pcl::PointXYZ>> clustered_obstacle_cloud;
    obstacle_clusterer->clusterObstacles(*obstacle_cloud, clustered_obstacle_cloud);

    const auto &obstacle_clustering_end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Obstacle clustering time: "
              << (obstacle_clustering_end_time - obstacle_clustering_start_time).count() / 1e9 << std::endl;

    std::cout << "Number of clusters: " << clustered_obstacle_cloud.size() << std::endl;

    // Polygonization
    const auto &convex_polygon_simplification_start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<geom::Point<float>>> cluster_outlines;
    // findOrderedConvexOutlines(clustered_obstacle_cloud, cluster_outlines);
    findOrderedConcaveOutlines(clustered_obstacle_cloud, cluster_outlines);

    const auto &convex_polygon_simplification_end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Convex polygon simplification time: "
              << (convex_polygon_simplification_end_time - convex_polygon_simplification_start_time).count() / 1e9
              << std::endl;

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
    else if (!ground_cloud_2->empty())
    {
        auto output_ground_segmentation_message = std::make_unique<PointCloud2>();
        output_ground_segmentation_message->header = input_message.header;
        output_ground_segmentation_message->is_bigendian = input_message.is_bigendian;
        convertPCLToPointCloud2(*ground_cloud_2, *output_ground_segmentation_message, 0, 255, 0);
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
    else if (!obstacle_cloud_2->empty())
    {
        auto output_obstacle_segmentation_message = std::make_unique<PointCloud2>();
        output_obstacle_segmentation_message->header = input_message.header;
        output_obstacle_segmentation_message->is_bigendian = input_message.is_bigendian;
        convertPCLToPointCloud2(*obstacle_cloud_2, *output_obstacle_segmentation_message, 255, 0, 0);
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

        // Convert to colorized point cloud
        auto colorized_cloud = convertClusteredCloudToColorizedCloud(clustered_obstacle_cloud);

        // Convert colorized cloud to PointCloud2
        convertPCLToPointCloud2(colorized_cloud, *output_clustered_obstacle_message);

        // Publish colorized clustered cloud
        publisher_obstacle_clustering_cloud_->publish(*output_clustered_obstacle_message);
    }

    // Convert to ROS2 format and publish (polygonization)
    if (!cluster_outlines.empty())
    {
        auto output_convex_polygonization_message = std::make_unique<MarkerArray>();
        convertPointXYZTypeToMarkerArray(cluster_outlines, input_message.header.frame_id, input_message.header.stamp,
                                         *output_convex_polygonization_message);
        publisher_obstacle_convex_hulls_->publish(*output_convex_polygonization_message);
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