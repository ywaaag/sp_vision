// include/armor_subscriber/omniperception.hpp

#ifndef OMNIPERCEPTION_HPP_
#define OMNIPERCEPTION_HPP_

#include <chrono>
#include <deque>
#include <mutex>
#include <std_msgs/msg/string.hpp>

#include "rclcpp/rclcpp.hpp"

namespace omniperception
{

struct TimestampedData
{
  std::string data;
  std::chrono::steady_clock::time_point timestamp;
};

class Omniperception : public rclcpp::Node
{
public:
  Omniperception();
  ~Omniperception();
  void start();
  std::optional<std::string> get_latest_data(std::chrono::steady_clock::time_point timestamp);

private:
  void topicCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::deque<TimestampedData> data_queue_;
  std::mutex data_mutex_;
};
}  // namespace omniperception

#endif  // OMNIPERCEPTION_HPP_