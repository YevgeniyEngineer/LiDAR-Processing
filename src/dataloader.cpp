// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// STL
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <string_view>
#include <tuple>
#include <vector>

using namespace std::chrono_literals;

class Dataloader final : public rclcpp::Node
{
  public:
    static constexpr const char *TOPIC = "pointcloud";
    static constexpr const char *FRAME_ID = "pointcloud";
    static constexpr const char *NODE_NAME = "dataloader";

    static constexpr auto PUBLICATION_RATE = 100ms;

    Dataloader(const std::filesystem::path &data_path);

  private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<sensor_msgs::msg::PointCloud2> preloaded_clouds_;
    std::vector<sensor_msgs::msg::PointCloud2>::iterator current_cloud_;
    std::chrono::steady_clock::time_point last_timestamp_;

    void convert(const pcl::PointCloud<pcl::PointXYZI> &cloud, sensor_msgs::msg::PointCloud2 &msg);
    void preload_point_clouds(const std::vector<std::filesystem::path> &file_paths);
    void publish_next_sample();
};

Dataloader::Dataloader(const std::filesystem::path &data_path) : rclcpp::Node(NODE_NAME)
{
    if (!std::filesystem::exists(data_path))
    {
        throw std::runtime_error("Specified data path does not exist: " + data_path.string());
    }

    std::vector<std::filesystem::path> files_paths;
    for (const auto &file_path : std::filesystem::directory_iterator(data_path))
    {
        if (file_path.path().extension() == ".pcd")
        {
            files_paths.push_back(file_path.path());
        }
    }

    if (files_paths.empty())
    {
        throw std::runtime_error("No files were loaded at all");
    }

    std::sort(files_paths.begin(), files_paths.end());

    preload_point_clouds(files_paths);

    rclcpp::QoS qos(2);
    qos.keep_last(2)
        .reliable()
        .durability_volatile()
        .liveliness(rclcpp::LivelinessPolicy::SystemDefault)
        .liveliness_lease_duration(std::chrono::seconds(1))
        .deadline(std::chrono::seconds(1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(TOPIC, qos);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(PUBLICATION_RATE),
                                     std::bind(&Dataloader::publish_next_sample, this));

    RCLCPP_INFO(this->get_logger(), "Dataloader node constructed. Publishing at a regular interval of %ld ms",
                PUBLICATION_RATE.count());
}

void Dataloader::convert(const pcl::PointCloud<pcl::PointXYZI> &cloud, sensor_msgs::msg::PointCloud2 &msg)
{
    static constexpr double SECONDS_TO_NANOSECONDS = 1.0e9;

    msg.header.frame_id = FRAME_ID;

    const auto nanosec_timestamp = std::chrono::steady_clock::now().time_since_epoch().count();
    msg.header.stamp.sec = static_cast<std::int32_t>(static_cast<double>(nanosec_timestamp / SECONDS_TO_NANOSECONDS));
    msg.header.stamp.nanosec = static_cast<std::uint32_t>(
        nanosec_timestamp -
        static_cast<std::int64_t>(static_cast<double>(msg.header.stamp.sec) * SECONDS_TO_NANOSECONDS));

    msg.height = cloud.height;
    msg.width = cloud.width;
    msg.is_bigendian = false;
    msg.point_step = sizeof(pcl::PointXYZI);
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = cloud.is_dense;

    static constexpr std::array<std::tuple<std::string_view, std::uint32_t, std::uint8_t, std::uint32_t>, 4> fields = {
        std::make_tuple("x", offsetof(pcl::PointXYZI, x), sensor_msgs::msg::PointField::FLOAT32, 1),
        std::make_tuple("y", offsetof(pcl::PointXYZI, y), sensor_msgs::msg::PointField::FLOAT32, 1),
        std::make_tuple("z", offsetof(pcl::PointXYZI, z), sensor_msgs::msg::PointField::FLOAT32, 1),
        std::make_tuple("intensity", offsetof(pcl::PointXYZI, intensity), sensor_msgs::msg::PointField::FLOAT32, 1)};

    msg.fields.clear();
    for (const auto &[name, offset, datatype, count] : fields)
    {
        sensor_msgs::msg::PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = datatype;
        field.count = count;
        msg.fields.push_back(std::move(field));
    }

    msg.data.resize(sizeof(pcl::PointXYZI) * cloud.size());
    std::memcpy(static_cast<void *>(msg.data.data()), static_cast<const void *>(cloud.data()),
                sizeof(pcl::PointXYZI) * cloud.size());
}

void Dataloader::preload_point_clouds(const std::vector<std::filesystem::path> &file_paths)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    cloud.points.reserve(200'000);
    cloud_msg.data.reserve(200'000 * sizeof(pcl::PointXYZI));
    preloaded_clouds_.reserve(file_paths.size());

    for (const auto &file_path : file_paths)
    {
        if (pcl::io::loadPCDFile(file_path, cloud) == -1)
        {
            RCLCPP_WARN(this->get_logger(), "Could not load .pcd file: %s", file_path.string().c_str());
            continue;
        }

        convert(cloud, cloud_msg);
        preloaded_clouds_.push_back(cloud_msg);
    }

    last_timestamp_ = std::chrono::steady_clock::now();
    current_cloud_ = preloaded_clouds_.begin();

    RCLCPP_INFO(this->get_logger(), "Loaded %lu data logs", preloaded_clouds_.size());
}

void Dataloader::publish_next_sample()
{
    if (current_cloud_ == preloaded_clouds_.end())
    {
        current_cloud_ = preloaded_clouds_.begin();
        auto last_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(last_timestamp_.time_since_epoch()).count();
        last_timestamp_ = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(
            last_ns + std::chrono::duration_cast<std::chrono::nanoseconds>(PUBLICATION_RATE).count()));
    }

    auto &msg = *current_cloud_;
    auto epoch_time_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(last_timestamp_.time_since_epoch()).count();
    msg.header.stamp = rclcpp::Time(epoch_time_ns);

    publisher_->publish(msg);
    last_timestamp_ += PUBLICATION_RATE;

    RCLCPP_INFO(this->get_logger(), "Published at [%ds, %uns]", msg.header.stamp.sec, msg.header.stamp.nanosec);
    ++current_cloud_;
}

std::int32_t main(std::int32_t argc, const char **const argv)
{
    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

    const auto data_path = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().append("data");

    auto ret = EXIT_SUCCESS;
    auto logger = rclcpp::get_logger("Point Cloud Publisher Main");

    try
    {
        const auto node = std::make_shared<Dataloader>(data_path);
        rclcpp::spin(node);
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(logger, "Exception: %s", ex.what());
        ret = EXIT_FAILURE;
    }
    catch (...)
    {
        RCLCPP_ERROR(logger, "Unknown exception");
        ret = EXIT_FAILURE;
    }

    rclcpp::shutdown();

    return ret;
}
