// STL
#include <chrono>     // std::chrono
#include <cstdint>    // std::uint32_t
#include <cstring>    // std::memcpy
#include <filesystem> // std::filesystem
#include <functional> // std::bind
#include <memory>     // std::shared_ptr
#include <stdexcept>  // std::runtime_error
#include <string>     // std::string
#include <tuple>      // std::tuple
#include <utility>    // std::move
#include <vector>     // std::vector

// ROS2
#include <rclcpp/executors.hpp>             // rclcpp::spin
#include <rclcpp/node.hpp>                  // rclcpp::Node
#include <rclcpp/publisher.hpp>             // rclcpp::Publisher
#include <rclcpp/qos.hpp>                   // rclcpp::QoS
#include <rclcpp/timer.hpp>                 // rclcpp::TimerBase
#include <rclcpp/utilities.hpp>             // rclcpp::shutdown
#include <sensor_msgs/msg/point_cloud2.hpp> // sensor_msgs::msg::PointCloud2
#include <sensor_msgs/msg/point_field.hpp>  // sensor_msgs::msg::PointField

// PCL
#include <pcl/PCLPointField.h> // pcl::PCLPointField::PointFieldTypes
#include <pcl/io/pcd_io.h>     // pcl::io::loadPCDFile
#include <pcl/point_cloud.h>   // pcl::PointCloud
#include <pcl/point_types.h>   // pcl::PointXYZI

using namespace std::chrono_literals;

namespace lidar_processing
{
class PointCloudPublisherNode : public rclcpp::Node
{
    using PointFieldTypes = pcl::PCLPointField::PointFieldTypes;
    using PointCloud2 = sensor_msgs::msg::PointCloud2;

    constexpr static float SENSOR_HEIGHT = 1.73F;
    constexpr static double SECONDS_TO_NANOSECONDS = 1.0e9;

  public:
    PointCloudPublisherNode(const std::filesystem::path &data_path, const std::string &topic = "pointcloud",
                            bool print_debug_info = false)
        : rclcpp::Node::Node("data_reader_node"), publisher_(nullptr), timer_(nullptr),
          print_debug_info_(print_debug_info)
    {
        if (!std::filesystem::exists(data_path))
        {
            throw std::runtime_error("Specified data path does not exist.");
        }

        std::cout << "data_reader_node started" << std::endl;

        // Specify QoS settings
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

        // Create publisher for PointCloud2 message type
        publisher_ = this->create_publisher<PointCloud2>(topic, qos);

        // Create event timer that will trigger readAndPublishDataSample every
        // 100ms
        timer_ = this->create_wall_timer(100ms, std::bind(&PointCloudPublisherNode::readAndPublishDataSample, this));

        // Read files
        for (const auto &file_path : std::filesystem::directory_iterator(data_path))
        {
            if (std::filesystem::is_regular_file(file_path) && file_path.path().extension() == ".pcd")
            {
                file_paths_.emplace_back(file_path.path());
            }
        }

        // Ensure data files are correctly ordered - sort lexicographically
        std::sort(file_paths_.begin(), file_paths_.end());

        // Start iterator from the first file name
        file_paths_iterator_ = file_paths_.begin();
    };

    virtual ~PointCloudPublisherNode() = default;

    void readAndPublishDataSample()
    {
        // Reset iterator if it reaches end
        if (file_paths_iterator_ == file_paths_.end())
        {
            file_paths_iterator_ = file_paths_.begin();
        }

        // Read point cloud
        const auto &file_path = (*file_paths_iterator_).string();
        pcl::PointCloud<pcl::PointXYZI> point_cloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_path, point_cloud) == -1)
        {
            throw std::runtime_error("Could not read .pcd file!");
        }

        // // Shift points to the sensor height
        // for (auto &point : point_cloud.points)
        // {
        //     point.z += SENSOR_HEIGHT;
        // }

        // Convert PointCloud to PointCloud2
        PointCloud2 output_message;
        output_message.header.frame_id = "pointcloud";

        // Get seconds + nanoseconds to populate metadata
        const auto nanosec_timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
        output_message.header.stamp.sec =
            static_cast<std::int32_t>(static_cast<double>(nanosec_timestamp / SECONDS_TO_NANOSECONDS));
        output_message.header.stamp.nanosec = static_cast<std::uint32_t>(
            nanosec_timestamp -
            static_cast<std::int64_t>(static_cast<double>(output_message.header.stamp.sec) * SECONDS_TO_NANOSECONDS));

        output_message.height = point_cloud.height;
        output_message.width = point_cloud.width;
        output_message.is_bigendian = false;
        output_message.point_step = sizeof(pcl::PointXYZI);
        output_message.row_step = sizeof(pcl::PointXYZI) * output_message.width;
        output_message.is_dense = point_cloud.is_dense;

        // Populate fields
        std::vector<std::tuple<std::string, std::uint32_t, std::uint8_t, std::uint32_t>> fields = {
            {"x", offsetof(pcl::PointXYZI, x), PointFieldTypes::FLOAT32, 1},
            {"y", offsetof(pcl::PointXYZI, y), PointFieldTypes::FLOAT32, 1},
            {"z", offsetof(pcl::PointXYZI, z), PointFieldTypes::FLOAT32, 1},
            {"intensity", offsetof(pcl::PointXYZI, intensity), PointFieldTypes::FLOAT32, 1}};

        for (const auto &field : fields)
        {
            sensor_msgs::msg::PointField field_cache;
            field_cache.name = std::get<0>(field);
            field_cache.offset = std::get<1>(field);
            field_cache.datatype = std::get<2>(field);
            field_cache.count = std::get<3>(field);
            output_message.fields.emplace_back(std::move(field_cache));
        }

        // Copy byte data
        output_message.data.resize(sizeof(pcl::PointXYZI) * point_cloud.size());
        std::memcpy(output_message.data.data(), point_cloud.data(), sizeof(pcl::PointXYZI) * point_cloud.size());

        // Publish PCLPointCloud2
        publisher_->publish(output_message);

        // Move to next file
        ++file_paths_iterator_;

        // Print debug message when the data was published
        if (print_debug_info_)
        {
            std::cout << "Published message at " << nanosec_timestamp << "\n";
        }
    }

  private:
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    std::vector<std::filesystem::path> file_paths_;
    std::vector<std::filesystem::path>::iterator file_paths_iterator_;
    bool print_debug_info_;
};
} // namespace lidar_processing

std::int32_t main(std::int32_t argc, const char **const argv)
{
    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

    const auto data_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().append("data");

    bool success = true;
    try
    {
        rclcpp::spin(std::make_shared<lidar_processing::PointCloudPublisherNode>(data_path));
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
