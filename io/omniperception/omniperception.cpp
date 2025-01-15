#include "omniperception.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <thread>

namespace io
{

Publish2Nav::Publish2Nav() : Node("auto_aim_target_pos_publisher")
{
  // 创建发布者，发布到 "auto_aim_target_pos" 话题
  publisher_ = this->create_publisher<std_msgs::msg::String>("auto_aim_target_pos", 10);

  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node initialized.");
}

Publish2Nav::~Publish2Nav()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node shutting down.");
}

// send_data 函数，接收 Eigen::Vector3d 类型的参数，并发布到话题
void Publish2Nav::send_data(const Eigen::Vector3d & target_pos)
{
  // 创建消息
  auto message = std::make_shared<std_msgs::msg::String>();

  // 将 Eigen::Vector3d 数据转换为字符串并存储在消息中
  message->data = "Target Position: [" + std::to_string(target_pos.x()) + ", " +
                  std::to_string(target_pos.y()) + ", " + std::to_string(target_pos.z()) + "]";

  // 发布消息
  publisher_->publish(*message);

  RCLCPP_INFO(this->get_logger(), "Sent message: '%s'", message->data.c_str());
}

// 启动节点，开始发布消息（如果需要的话）
void Publish2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "Starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

}  // namespace io
