#include "obstacle_clustering_node.hpp"
#include "point_labels.hpp"
namespace lidar_processing
{
using std::placeholders::_1;

ObstacleClusteringNode::ObstacleClusteringNode() : Node("obstacle_clustering_node")
{
    std::cout << "ObstacleClusteringNode node started." << std::endl;

    // Subscriber will receive a message from data_reader_node
    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "obstacle_pointcloud", 10, std::bind(&ObstacleClusteringNode::clusterObstacles, this, _1));

    // Publisher will publish a message whenever subcription callback is triggered
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_clustered", 10);
}

void ObstacleClusteringNode::clusterObstacles(const sensor_msgs::msg::PointCloud2 &ros2_message)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", ros2_message.header.frame_id.c_str());

    // Convert sensor_msgs::msg::PointCloud2 to pcl::PointCloud<pcl::PointXYZRGBI>
    pcl::PCLPointCloud2::Ptr pcl_message = std::make_shared<pcl::PCLPointCloud2>();
    pcl::PointCloud<pcl::PointXYZRGBI>::Ptr pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBI>>();

    convert(ros2_message, *pcl_message);
    convert(*pcl_message, *pcl_cloud);

    // Apply clustering and label clustered cloud (different color for each cluster)
    std::shared_ptr<ObstacleClustering> obstacle_clustering = std::make_shared<ObstacleClustering>();
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr clustered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
    obstacle_clustering->clusterObstacles(*pcl_cloud, *clustered_cloud);

    sensor_msgs::msg::PointCloud2 clustered_cloud_message;
    convert(*clustered_cloud, clustered_cloud_message);
    clustered_cloud_message.header.stamp = ros2_message.header.stamp;
    clustered_cloud_message.header.frame_id = ros2_message.header.frame_id;
    publisher_->publish(clustered_cloud_message);
}
} // namespace lidar_processing

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();
    rclcpp::spin(std::make_shared<lidar_processing::ObstacleClusteringNode>());
    rclcpp::shutdown();
    return 0;
}