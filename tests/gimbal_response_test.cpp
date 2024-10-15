#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{delta-angle a  |         45          | yaw轴delta角}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  auto delta_angle = cli.get<double>("delta-angle");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::CBoard cboard(config_path);

  auto init_t = std::chrono::steady_clock::now();

  std::this_thread::sleep_for(1ms);
  Eigen::Quaterniond init_q = cboard.imu_at(init_t);
  auto init_yaw = (tools::eulers(init_q, 2, 1, 0) * 57.3)[0];

  auto dyaw = delta_angle / 500;
  double cmd_yaw = init_yaw;
  int i = 0;

  while (!exiter.exit()) {
    io::Command command;
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;

    if (i == 500) {
      i = 0;
      cmd_yaw = init_yaw;
    }

    cmd_yaw += dyaw;
    command = {1, 0, cmd_yaw / 57.3, 0};
    cboard.send(command);
    i++;
    tools::logger()->info("cmd_yaw is {:.4f}", cmd_yaw / 57.3);
  }

  return 0;
}