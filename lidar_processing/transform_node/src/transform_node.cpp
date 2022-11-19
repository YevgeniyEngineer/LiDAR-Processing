#include "transform_node.hpp"

namespace lidar_processing
{
PosePublisher::PosePublisher()
    : Node("pose_publisher_node"), tx_(0.0F), ty_(0.0F), tz_(0.0F), rx_(0.0F), ry_(0.0F), rz_(0.0F)
{
    std::cout << "PosePublisher node started." << std::endl;
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    PosePublisher::poseCallback();

    // publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("pose", 10);
    // timer_ = this->create_wall_timer(std::chrono::microseconds(50), std::bind(&PosePublisher::poseCallback, this));
}

PosePublisher::PosePublisher(float tx, float ty, float tz, float rx, float ry, float rz)
    : Node("pose_publisher_node"), tx_(tx), ty_(ty), tz_(tz), rx_(rx), ry_(ry), rz_(rz)
{
    std::cout << "PosePublisher node started." << std::endl;
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    PosePublisher::poseCallback();

    // publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("pose", 10);
    // timer_ = this->create_wall_timer(std::chrono::microseconds(50), std::bind(&PosePublisher::poseCallback, this));
}

void PosePublisher::poseCallback()
{
    // transform
    geometry_msgs::msg::TransformStamped transform_stamped_;

    transform_stamped_.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
    transform_stamped_.header.frame_id = "world";
    transform_stamped_.child_frame_id = "pointcloud";
    transform_stamped_.transform.translation.x = tx_;
    transform_stamped_.transform.translation.y = ty_;
    transform_stamped_.transform.translation.z = tz_;

    tf2::Quaternion quaternion_;
    quaternion_.setRPY(rx_, ry_, rz_);
    transform_stamped_.transform.rotation.x = quaternion_.x();
    transform_stamped_.transform.rotation.y = quaternion_.y();
    transform_stamped_.transform.rotation.z = quaternion_.z();
    transform_stamped_.transform.rotation.w = quaternion_.w();

    tf_static_broadcaster_->sendTransform(transform_stamped_);
    std::cout << "Translation: (" << transform_stamped_.transform.translation.x << ", "
              << transform_stamped_.transform.translation.y << ", " << transform_stamped_.transform.translation.z << ")"
              << std::endl
              << "Rotation: (" << transform_stamped_.transform.rotation.x << ", "
              << transform_stamped_.transform.rotation.y << ", " << transform_stamped_.transform.rotation.z << ", "
              << transform_stamped_.transform.rotation.w << ")" << std::endl;
}
} // namespace lidar_processing

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();
    rclcpp::spin(std::make_shared<lidar_processing::PosePublisher>());
    rclcpp::shutdown();
    return 0;
}