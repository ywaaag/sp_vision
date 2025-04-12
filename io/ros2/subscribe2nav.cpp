#include "subscribe2nav.hpp"

#include <sstream>
#include <vector>

namespace io
{

Subscribe2Nav::Subscribe2Nav()
: Node("nav_subscriber"), enemy_statue_queue_(1), autoaim_target_queue_(1)
{
  enemy_status_subscription_ = this->create_subscription<sp_msgs::msg::EnemyStatusMsg>(
    "enemy_status", 10,
    std::bind(&Subscribe2Nav::enemy_status_callback, this, std::placeholders::_1));

  autoaim_target_subscription_ = this->create_subscription<sp_msgs::msg::AutoaimTargetMsg>(
    "autoaim_target", 10,
    std::bind(&Subscribe2Nav::autoaim_target_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "nav_subscriber node initialized.");
}

Subscribe2Nav::~Subscribe2Nav()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node shutting down.");
}

void Subscribe2Nav::enemy_status_callback(const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg)
{
  enemy_statue_queue_.clear();
  enemy_statue_queue_.push(*msg);
}

void Subscribe2Nav::autoaim_target_callback(const sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg)
{
  autoaim_target_queue_.clear();
  autoaim_target_queue_.push(*msg);
}

void Subscribe2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node Starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

std::vector<int8_t> Subscribe2Nav::subscribe_enemy_status()
{
  if (enemy_statue_queue_.empty()) {
    // RCLCPP_INFO(this->get_logger(), "No enemy_status detected !");
    return std::vector<int8_t>();
  }
  sp_msgs::msg::EnemyStatusMsg msg;

  enemy_statue_queue_.back(msg);
  RCLCPP_INFO(
    this->get_logger(), "Subscribe enemy_status at: %d.%09u", msg.timestamp.sec,
    msg.timestamp.nanosec);

  return msg.invincible_enemy_ids;
}

std::vector<int8_t> Subscribe2Nav::subscribe_autoaim_target()
{
  if (autoaim_target_queue_.empty()) {
    // RCLCPP_INFO(this->get_logger(), "No autoaim_target detected !");
    return std::vector<int8_t>();
  }
  sp_msgs::msg::AutoaimTargetMsg msg;

  autoaim_target_queue_.back(msg);
  RCLCPP_INFO(
    this->get_logger(), "Subscribe autoaim_target at: %d.%09u", msg.timestamp.sec,
    msg.timestamp.nanosec);

  return msg.target_ids;
}

}  // namespace io
