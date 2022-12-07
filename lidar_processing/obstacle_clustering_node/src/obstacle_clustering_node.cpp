#include "obstacle_clustering_node.hpp"
#include "convex_hull.hpp"
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
    publisher_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clustered_pointcloud", 10);
    publisher_marker_array_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_lines", 10);
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

    auto t1 = std::chrono::high_resolution_clock::now();
    obstacle_clustering->clusterObstacles(*pcl_cloud, *clustered_cloud);
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Elapsed time (clustering): "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.0 << "s" << std::endl;

    sensor_msgs::msg::PointCloud2 clustered_cloud_message;
    convert(*clustered_cloud, clustered_cloud_message);
    clustered_cloud_message.header.stamp = ros2_message.header.stamp;
    clustered_cloud_message.header.frame_id = ros2_message.header.frame_id;
    publisher_cloud_->publish(clustered_cloud_message);

    // Apply polygonization
    // Split into clusters -> std::vector<std::vector<point_t>> point_t : ( x, y )
    std::vector<std::deque<point_t>> clusters(1);
    std::uint32_t current_rgba = clustered_cloud->points[0].rgba;
    int cluster_index = 0;
    for (const auto &point : clustered_cloud->points)
    {
        if (point.rgba != current_rgba)
        {
            current_rgba = point.rgba;
            clusters.push_back({});
            ++cluster_index;
        }
        point_t new_2d_point;
        new_2d_point.x = point.x;
        new_2d_point.y = point.y;
        // std::cout << "new 2d point: (" << point.x << ", " << point.y << ") \n";
        clusters[cluster_index].push_back(std::move(new_2d_point));
    }
    std::cout << "Number of clusters: " << clusters.size() << std::endl;

    t1 = std::chrono::high_resolution_clock::now();

    visualization_msgs::msg::MarkerArray polygons;
    polygons.markers.reserve(clusters.size());
    uint32_t polygon_id = 0U;
    for (const auto &cluster_points : clusters)
    {
        ConvexHull convex_hull(cluster_points);
        int number_of_hull_points = static_cast<int>(cluster_points.size());
        auto cluster_hull = convex_hull.getResultAsArray(number_of_hull_points);
        // std::cout << "Constructed convex hull with " << number_of_hull_points << " points from original "
        //           << cluster_points.size() << " points\n";

        visualization_msgs::msg::Marker polygon;

        polygon.lifetime.sec = 0.0;
        polygon.lifetime.nanosec = 0.1 * 1e9;
        polygon.header.frame_id = clustered_cloud_message.header.frame_id;
        polygon.header.stamp = clustered_cloud_message.header.stamp;
        polygon.ns = "lidar_processing";
        polygon.id = polygon_id;
        ++polygon_id;

        polygon.type = visualization_msgs::msg::Marker::LINE_STRIP;
        polygon.action = visualization_msgs::msg::Marker::ADD;
        polygon.scale.x = 0.15;
        polygon.color.a = 1.0;
        polygon.color.r = 1.0;
        polygon.color.g = 0.0;
        polygon.color.b = 1.0;

        polygon.pose.position.x = 0.0;
        polygon.pose.position.y = 0.0;
        polygon.pose.position.z = 0.0;
        polygon.pose.orientation.x = 0.0;
        polygon.pose.orientation.y = 0.0;
        polygon.pose.orientation.z = 0.0;
        polygon.pose.orientation.w = 1.0;

        polygon.points.reserve(cluster_hull.size());
        for (int i = 0; i < cluster_hull.size(); ++i)
        {
            geometry_msgs::msg::Point polygon_point;
            polygon_point.x = cluster_hull[i].x;
            polygon_point.y = cluster_hull[i].y;
            polygon_point.z = 0.0;
            polygon.points.push_back(std::move(polygon_point));
        }
        polygons.markers.push_back(std::move(polygon));
    }
    publisher_marker_array_->publish(polygons);

    t2 = std::chrono::high_resolution_clock::now();
    std::cout << "Elapsed time (polygonization): "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.0 << "s" << std::endl;
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