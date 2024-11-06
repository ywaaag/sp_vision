#include "omniperception.hpp"

namespace omniperception
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
}

void Omniperception::start()
{
  RCLCPP_INFO(this->get_logger(), "Starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

void Omniperception::topicCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // 将新消息添加到队列中，并保存时间戳
  auto timestamp = std::chrono::steady_clock::now();
  data_queue_.emplace_back(TimestampedData{msg->data, timestamp});

  // 限制队列大小，避免无限增长
  if (data_queue_.size() > 100) {
    data_queue_.pop_front();
  }

  RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
}

// 根据传入的时间戳获取最新数据
std::optional<std::string> Omniperception::get_latest_data(
  std::chrono::steady_clock::time_point timestamp)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // 从队列中找到最接近传入时间戳的消息
  for (auto it = data_queue_.rbegin(); it != data_queue_.rend(); ++it) {
    if (it->timestamp <= timestamp) {
      return it->data;
    }
  }
  return std::nullopt;  // 如果没有找到合适的数据，返回空
}

}  // namespace omniperception
