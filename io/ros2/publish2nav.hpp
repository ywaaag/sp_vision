#ifndef IO__PBLISH2NAV_HPP
#define IO__PBLISH2NAV_HPP

#include <Eigen/Dense>  // For Eigen::Vector3d
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace io
{
class Publish2Nav : public rclcpp::Node
{
public:
  // 构造函数：初始化订阅者和发布者
  Publish2Nav();

  // 析构函数
  ~Publish2Nav();

  // 启动节点的事件循环
  void start();

  // 发送Eigen::Vector3d数据到话题
  void send_data(const Eigen::Vector3d & data);

private:
  // ROS2 发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

}  // namespace io

#endif  // Publish2Nav_HPP_
