#include "ros2.hpp"
namespace io
{
ROS2::ROS2()
{
  rclcpp::init(0, nullptr);

  publish2nav_ = std::make_shared<Publish2Nav>();

  spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });
}

ROS2::~ROS2()
{
  rclcpp::shutdown();
  spin_thread_->join();
}

void ROS2::publish(const Eigen::Vector3d & target_pos) { publish2nav_->send_data(target_pos); }

}  // namespace io
