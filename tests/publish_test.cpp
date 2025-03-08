#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim_sentry/armor.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;

  double i = 0;
  while (!exiter.exit()) {
    i++;
    auto timestamp = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    Eigen::Vector4d data{i, i + 1, 1, auto_aim::ArmorName::sentry + 1};

    ros2.publish(data);
    auto x = ros2.subscribe();
    tools::logger()->info("Received: ", x.size());
    if (i > 1000) break;
  }

  return 0;
}
