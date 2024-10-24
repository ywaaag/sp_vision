#include "io/omniperception/omniperception.hpp"

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "tools/exiter.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<omniperception_subscriber::Omniperception>();

  std::thread spin_thread([node]() { node->start(); });

  // 模拟等待若干秒后获取数据
  std::this_thread::sleep_for(std::chrono::seconds(5));
  int i = 0;
  while (!exiter.exit()) {
    i++;
    auto timestamp = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::seconds(1));  // 等待更多数据到达

    auto data = node->get_latest_data(timestamp);
    if (data) {
      RCLCPP_INFO(node->get_logger(), "Latest data since timestamp: '%s'", data->c_str());
    } else {
      RCLCPP_INFO(node->get_logger(), "No data found since the given timestamp.");
    }
    if (i > 30000) break;
  }
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
