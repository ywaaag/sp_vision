#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/pid.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |     | 输出命令行参数说明}"
  "{w              | 5.0 | yaw速度目标值(rad/s)}"
  "{p              | 1.0 | P项系数}"
  "{i              | 1.0 | I项系数}"
  "{d              | 0.0 | D项系数}"
  "{max-out        | 7.0 | PID最大输出值}"
  "{max-iout       | 1.0 | I项最大输出值}"
  "{@config-path   |     | yaml配置文件路径}";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  auto vyaw_set = cli.get<double>("w");
  auto p = cli.get<double>("p");
  auto i = cli.get<double>("i");
  auto d = cli.get<double>("d");
  auto max_out = cli.get<double>("max-out");
  auto max_iout = cli.get<double>("max-iout");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::PID vyaw_pid(0.01, p, i, d, max_out, max_iout);

  io::Gimbal gimbal(config_path);

  auto t0 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    auto state = gimbal.state();
    auto yaw_torque = vyaw_pid.calc(vyaw_set, state.vyaw);
    gimbal.send(true, false, state.yaw, state.vyaw, yaw_torque, state.pitch, state.vpitch, 0);

    nlohmann::json data;
    data["gimbal_yaw"] = state.yaw;
    data["gimbal_vyaw"] = state.vyaw;
    data["gimbal_pitch"] = state.pitch;
    data["gimbal_vpitch"] = state.vpitch;
    data["bullet_speed"] = state.bullet_speed;
    data["vyaw_set"] = vyaw_set;
    data["yaw_torque"] = yaw_torque;
    data["iout"] = vyaw_pid.iout;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
    plotter.plot(data);

    std::this_thread::sleep_for(10ms);
  }

  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}