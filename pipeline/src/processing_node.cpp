// Processing
#include "ground_segmentation.hpp"

// STL
#include <chrono>
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
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace lidar_processing
{
class ProcessingNode : public rclcpp::Node
{
  public:
    ProcessingNode() : rclcpp::Node::Node("processing_node")
    {
        std::cout << "processing_node started" << std::endl;
    }

    ~ProcessingNode() = default;

  private:
    // Subscriber receives raw point cloud data
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

    // Ground segmentation
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_ground_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_obstacle_cloud_;

    // Obstacle clustering
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_obstacle_clustering_cloud_;

    // Obstacle cluster polygonization
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_obstacle_convex_hulls_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_obstacle_concave_hulls_;
};
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