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
    sensor_msgs::msg::PointCloud2::Ptr message = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // Reset iterator
    if (filenames_iterator_ == filenames_.end())
    {
        filenames_iterator_ = filenames_.begin();
    }

    // Read pointcloud
    filename_ = (*filenames_iterator_).string();
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_message = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename_, *pcl_message) == -1)
    {
        std::cout << "Clouldn't read .pcd file" << std::endl;
        return;
    }
    else
    {
        std::cout << "Successfully read " << filename_ << std::endl;
    }

    // pcl::PointCloud to sensor_msgs::msg::PointCloud2
    message->fields.clear();
    if (pcl_message->width == 0 && pcl_message->height == 0)
    {
        message->width = pcl_message->size();
        message->height = 1;
    }
    else
    {
        assert(pcl_message->size() == pcl_message->width * pcl_message->height);
        message->height = pcl_message->height;
        message->width = pcl_message->width;
    }

    // Fill point cloud binary data (padding and all)
    std::size_t data_size = sizeof(pcl::PointXYZI) * pcl_message->size();
    message->data.resize(data_size);
    if (data_size)
    {
        std::memcpy(&(message->data[0]), &(*pcl_message)[0], data_size);
    }

    // Fill metadata
    message->header.frame_id = "lidar_transform";
    message->point_step = sizeof(pcl::PointXYZI);
    message->row_step = sizeof(pcl::PointXYZI) * message->width;
    message->is_dense = pcl_message->is_dense;
    message->is_bigendian = false;

    // Publish poincloud
    publisher_->publish(*message);

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