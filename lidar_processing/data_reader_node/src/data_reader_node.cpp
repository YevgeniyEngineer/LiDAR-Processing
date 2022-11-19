#include "data_reader_node.hpp"

namespace lidar_processing
{
PointCloudPublisher::PointCloudPublisher() : Node("point_cloud_publisher_node")
{
    std::cout << "PointCloudPublisher node started." << std::endl;

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PointCloudPublisher::timerCallback, this));

    filenames_ = readFilenamesExt(filepath, ".pcd");
    filenames_iterator_ = filenames_.begin();
}

std::vector<std::filesystem::path> PointCloudPublisher::readFilenamesExt(std::filesystem::path const &root,
                                                                         std::string const &ext)
{
    std::vector<std::filesystem::path> paths = {};

    // find files
    if (std::filesystem::exists(root) && std::filesystem::is_directory(root))
    {
        for (const auto &entry : std::filesystem::recursive_directory_iterator(root))
        {
            if (std::filesystem::is_regular_file(entry) && entry.path().extension() == ext)
                paths.emplace_back(entry.path());
        }
    }

    // sort names lexicographically
    if (!paths.empty())
    {
        std::sort(paths.begin(), paths.end());
    }

    return paths;
}

void PointCloudPublisher::timerCallback()
{
    // Reset iterator
    if (filenames_iterator_ == filenames_.end())
    {
        filenames_iterator_ = filenames_.begin();
    }

    // Read pointcloud
    filename_ = (*filenames_iterator_).string();
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename_, *pcl_cloud) == -1)
    {
        std::cout << "Clouldn't read .pcd file" << std::endl;
        return;
    }
    else
    {
        std::cout << "Successfully read " << filename_ << std::endl;
    }

    // Convert pcl::PointCloud<pcl::PointXYZI> to binary pcl::PointCloud2 format
    pcl::PCLPointCloud2::Ptr pcl_message = std::make_shared<pcl::PCLPointCloud2>();
    pcl::toPCLPointCloud2(*pcl_cloud, *pcl_message);
    pcl_message->header.frame_id = "pointcloud";

    // Copy binary blob pcl::PCLPointCloud2 to sensor_msgs::msg::PointCloud2
    sensor_msgs::msg::PointCloud2::Ptr ros2_message = std::make_shared<sensor_msgs::msg::PointCloud2>();

    ros2_message->header.frame_id = pcl_message->header.frame_id;

    // pcl message: the timestamp uint64_t value represents microseconds since 1970-01-01 00:00:00 (the UNIX epoch).
    auto timestamp = std::chrono::system_clock::now().time_since_epoch();
    int32_t seconds = timestamp / std::chrono::seconds(1);
    uint32_t nanoseconds = timestamp / std::chrono::nanoseconds(1);

    ros2_message->header.stamp.sec = seconds;
    ros2_message->header.stamp.nanosec = nanoseconds;

    ros2_message->height = pcl_message->height;
    ros2_message->width = pcl_message->width;

    for (const auto field : pcl_message->fields)
    {
        std::cout << "field_name: " << field.name << std::endl;
        std::cout << "offset: " << field.offset << std::endl;
        std::cout << "datatype: " << field.datatype << std::endl;
        std::cout << "count: " << field.count << std::endl;

        sensor_msgs::msg::PointField ros2_field;
        ros2_field.name = field.name;
        ros2_field.offset = field.offset;
        ros2_field.datatype = field.datatype;
        ros2_field.count = field.count;
        ros2_message->fields.emplace_back(ros2_field);
    }

    ros2_message->is_bigendian = pcl_message->is_bigendian;
    ros2_message->point_step = pcl_message->point_step;
    ros2_message->row_step = pcl_message->row_step;
    ros2_message->data = pcl_message->data;
    ros2_message->is_dense = pcl_message->is_dense;

    // Print message info
    std::cout << "frame_id: " << ros2_message->header.frame_id << std::endl;
    std::cout << "sec: " << ros2_message->header.stamp.sec << std::endl;
    std::cout << "nanosec: " << ros2_message->header.stamp.nanosec << std::endl;
    std::cout << "height: " << ros2_message->height << std::endl;
    std::cout << "width: " << ros2_message->width << std::endl;
    std::cout << "is_bigendian: " << ros2_message->is_bigendian << std::endl;
    std::cout << "point_step: " << ros2_message->point_step << std::endl;
    std::cout << "row_step: " << ros2_message->row_step << std::endl;
    std::cout << "is_dense: " << ros2_message->is_dense << std::endl;

    // Publish poincloud
    publisher_->publish(*ros2_message);

    // Increment iterator
    ++filenames_iterator_;
}
} // namespace lidar_processing

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();
    rclcpp::spin(std::make_shared<lidar_processing::PointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}