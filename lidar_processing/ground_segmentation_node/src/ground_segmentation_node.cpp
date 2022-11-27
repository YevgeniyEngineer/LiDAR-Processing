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

    // Publisher for visualisation
    publisher_visualisation_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_segmented_visualisation", 10);
}

void GroundSegmentationNode::segmentGround(const sensor_msgs::msg::PointCloud2 &ros2_message)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", ros2_message.header.frame_id.c_str());

    // Convert sensor_msgs::msg::PointCloud2 to pcl::PointCloud<pcl::PointXYZI>
    pcl::PCLPointCloud2::Ptr pcl_message = std::make_shared<pcl::PCLPointCloud2>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    convert(ros2_message, *pcl_message);
    convert(*pcl_message, *pcl_cloud);

    // Apply segmentation and label segmented cloud as ground and nonground points
    std::shared_ptr<GroundSegmentation> ground_segmetation = std::make_shared<GroundSegmentation>();

    pcl::PointCloud<pcl::PointXYZRGBI>::Ptr pcl_cloud_segmented =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGBI>>();

    auto t1 = std::chrono::high_resolution_clock::now();
    ground_segmetation->segmentGround(*pcl_cloud, *pcl_cloud_segmented);
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Ground segmentation took: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.0F << " seconds"
              << std::endl;

    // Convert and publish message (for further processing)
    sensor_msgs::msg::PointCloud2::Ptr output_message = std::make_shared<sensor_msgs::msg::PointCloud2>();
    convert(*pcl_cloud_segmented, *output_message);
    output_message->header.stamp = ros2_message.header.stamp;
    output_message->header.frame_id = ros2_message.header.frame_id;

    publisher_->publish(*output_message);
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