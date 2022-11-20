#include "ground_segmentation_node.hpp"

namespace lidar_processing
{
using std::placeholders::_1;

GroundSegmentationNode::GroundSegmentationNode() : Node("ground_segmentation_node")
{
    std::cout << "GroundSegmentationNode node started." << std::endl;

    // Subscriber will receive a message from data_reader_node
    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud", 10, std::bind(&GroundSegmentationNode::segmentGround, this, _1));

    // Publisher will publish a message whenever subcription callback is triggered
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_segmented", 10);
}

void GroundSegmentationNode::segmentGround(const sensor_msgs::msg::PointCloud2 &input_message)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", input_message.header.frame_id.c_str());

    // Convert input_message to pcl::PointCloud<pcl::PointXYZI>

    // Apply segmentation and label segmented cloud as ground and nonground points

    sensor_msgs::msg::PointCloud2::Ptr output_message = std::make_shared<sensor_msgs::msg::PointCloud2>();
}
} // namespace lidar_processing

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();
    rclcpp::spin(std::make_shared<lidar_processing::GroundSegmentationNode>());
    rclcpp::shutdown();
    return 0;
}