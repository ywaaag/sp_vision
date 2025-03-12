#ifndef IO__SUBSCRIBE2NAV_HPP
#define IO__SUBSCRIBE2NAV_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "sp_msgs/msg/enemy_status_msg.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class Subscribe2Nav : public rclcpp::Node
{
public:
  Subscribe2Nav();

  ~Subscribe2Nav();

  void start();

  std::vector<int8_t> subscribe_data();

private:
  void callback(const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg);

  rclcpp::Subscription<sp_msgs::msg::EnemyStatusMsg>::SharedPtr subscription_;

  tools::ThreadSafeQueue<sp_msgs::msg::EnemyStatusMsg> queue_;
};
}  // namespace io

#endif  // IO__SUBSCRIBE2NAV_HPP
