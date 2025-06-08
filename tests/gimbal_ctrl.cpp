#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/pid.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }"
  "{@kp            | | }"
  "{@ki            | | }"
  "{@kd            | | }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  auto kp = cli.get<float>("@kp");
  auto ki = cli.get<float>("@ki");
  auto kd = cli.get<float>("@kd");

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Gimbal gimbal(config_path);

  tools::PID yaw_pid(1e-2, kp, ki, kd, 7, 1, true);
  tools::PID pitch_pid(1e-2, 5, 0, 0.1, 7, 1, true);

  auto count = 0;
  auto yaw_set = 0.0;
  auto pitch_set = 0.0;
  auto t0 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    if (count % 100 == 0) {
      count = 0;
      yaw_set = tools::limit_rad(yaw_set + 10.0 / 57.3);
    }

    auto yaw = gimbal.yaw;
    auto vyaw = gimbal.vyaw;
    auto pitch = gimbal.pitch;
    auto vpitch = gimbal.vpitch;

    auto yaw_torque = yaw_pid.calc(yaw_set, yaw);
    auto pitch_torque = pitch_pid.calc(pitch_set, pitch);
    gimbal.send(yaw, vyaw, yaw_torque, pitch, vpitch, pitch_torque);

    nlohmann::json data;
    data["yaw"] = yaw;
    data["vyaw"] = vyaw;
    data["pitch"] = pitch;
    data["vpitch"] = vpitch;
    data["yaw_set"] = yaw_set;
    data["yaw_torque"] = yaw_torque;
    data["pitch_torque"] = pitch_torque;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
    plotter.plot(data);

    count++;
    std::this_thread::sleep_for(10ms);
  }

  gimbal.send(0, 0, 0, 0, 0, 0);

  return 0;
}