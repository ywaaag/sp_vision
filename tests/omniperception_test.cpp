#include "io/omniperception/omniperception.hpp"

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "tools/exiter.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<io::Publish2Nav>();

  std::thread spin_thread([node]() { node->start(); });

  double i = 0;
  while (!exiter.exit()) {
    i++;
    auto timestamp = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::seconds(1));  // 等待更多数据到达
    Eigen::Vector3d data{i, i + 1, i + 2};

    node->send_data(data);
    if (i > 100) break;
  }
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
