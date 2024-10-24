// include/armor_subscriber/omniperception.hpp

#ifndef OMNIPERCEPTION_HPP_
#define OMNIPERCEPTION_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"  // 确保这一行存在

namespace omniperception_subscriber
{

class Omniperception : public rclcpp::Node
{
public:
  Omniperception();
  ~Omniperception();

  void start();

private:
  void topicCallback(const std_msgs::msg::String::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string data_;
  std::mutex data_mutex_;
};

}  // namespace omniperception_subscriber

#endif  // OMNIPERCEPTION_HPP_