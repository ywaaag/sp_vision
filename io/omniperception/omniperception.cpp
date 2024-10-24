// src/omniperception.cpp

#include "omniperception.hpp"

namespace omniperception_subscriber
{

Omniperception::Omniperception() : Node("armor_subscriber")
{
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic_name", 10, std::bind(&Omniperception::topicCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "armor_subscriber node initialized.");
}

Omniperception::~Omniperception()
{
  RCLCPP_INFO(this->get_logger(), "armor_subscriber node shutting down.");
  // Cleanup if necessary
}

void Omniperception::start()
{
  RCLCPP_INFO(this->get_logger(), "Starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

void Omniperception::topicCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "Received message: '%s'", data_.c_str());
  // Process message here
}

}  // namespace omniperception_subscriber