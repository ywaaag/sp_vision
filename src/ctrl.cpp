#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/cboard.hpp"
#include "tasks/auto_aim/solver.hpp"
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

  io::CBoard cboard(config_path);
  auto_aim::Solver solver(config_path);

  tools::PID yaw_pid(1e-2, kp, ki, kd, 7, 1, true);

  auto t0 = std::chrono::steady_clock::now();

  auto count = 0;
  auto yaw_set = 0.0f;

  while (!exiter.exit()) {
    if (count % 300 == 0) {
      count = 0;
      yaw_set = tools::limit_rad(yaw_set + M_PI / 4.0f);
    }

    auto t = std::chrono::steady_clock::now();
    Eigen::Quaterniond q = cboard.imu_at(t);

    Eigen::Vector3d ypr = tools::eulers(q, 2, 1, 0);

    auto out = yaw_pid.calc(yaw_set, ypr[0]);
    io::Command cmd{false, false, out / 10, 0.0};
    cboard.send(cmd);

    nlohmann::json data;
    data["yaw"] = ypr[0];
    data["pitch"] = ypr[1];
    data["roll"] = ypr[2];
    data["yaw_set"] = yaw_set;
    data["out"] = out;
    data["t"] = tools::delta_time(t, t0);
    plotter.plot(data);

    count++;
    std::this_thread::sleep_for(10ms);
  }

  return 0;
}