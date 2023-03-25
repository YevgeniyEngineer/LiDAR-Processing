// Processing
#include "conversions.hpp"
#include "dbscan.hpp"
#include "ground_segmentation.hpp"
#include "internal_types.hpp"
#include "obstacle_clustering.hpp"
#include "obstacle_simplification.hpp"

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
        rclcpp::QoS qos(5);
        qos.deadline(std::chrono::seconds(1));
        qos.liveliness_lease_duration(std::chrono::seconds(1));
        qos.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE);
        qos.history(rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        subscriber_ = this->create_subscription<PointCloud2>(
            "pointcloud", qos, std::bind(&ProcessingNode::process, this, std::placeholders::_1));

        // Publisher nodes
        publisher_ground_cloud_ = this->create_publisher<PointCloud2>("ground_pointcloud", qos);
        publisher_obstacle_cloud_ = this->create_publisher<PointCloud2>("obstacle_pointcloud", qos);
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
};

void ProcessingNode::process(const PointCloud2 &input_message)
{
    // Instantiate processing objects
    std::unique_ptr<GroundSegmenter> ground_segmenter = std::make_unique<GroundSegmenter>();
    std::unique_ptr<ObstacleClusterer> obstacle_clusterer = std::make_unique<ObstacleClusterer>();

    obstacle_clusterer->setNeighbourRadiusThreshold(0.5);
    obstacle_clusterer->setMinClusterSize(10);
    obstacle_clusterer->setClusteringAlgorithm(ClusteringAlgorithm::FAST_EUCLIDEAN_CLUSTERING);

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

    std::vector<pcl::PointCloud<pcl::PointXYZ>> clustered_obstacle_cloud;
    obstacle_clusterer->clusterObstacles(*obstacle_cloud, clustered_obstacle_cloud);

    const auto &obstacle_clustering_end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Obstacle clustering time: "
              << (obstacle_clustering_end_time - obstacle_clustering_start_time).count() / 1e9 << std::endl;

    std::cout << "Number of clusters: " << clustered_obstacle_cloud.size() << std::endl;

    // Polygonization
    const auto &convex_polygon_simplification_start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<PointXY>> convex_hulls;
    findOrderedConvexOutline(clustered_obstacle_cloud, convex_hulls);

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
    if (!convex_hulls.empty())
    {
        std::unique_ptr<MarkerArray> output_convex_polygonization_message = std::make_unique<MarkerArray>();
        convertPointXYZTypeToMarkerArray(convex_hulls, input_message.header.frame_id, input_message.header.stamp,
                                         *output_convex_polygonization_message);
        publisher_obstacle_convex_hulls_->publish(*output_convex_polygonization_message);
    }
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