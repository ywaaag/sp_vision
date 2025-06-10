#include <Eigen/Dense>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>("@config-path");

  auto yaml = tools::load(config_path);
  const auto dt = tools::read<double>(yaml, "dt");
  const auto inertia = tools::read<double>(yaml, "yaw_inertia");
  const auto damping = tools::read<double>(yaml, "yaw_damping");
  const auto torque_set = tools::read<double>(yaml, "yaw_torque_set");

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Gimbal gimbal(config_path);
  auto t0 = std::chrono::steady_clock::now();

  auto state = gimbal.state();
  for (int i = 0; i < 100; ++i) {
    std::this_thread::sleep_for(10ms);
    state = gimbal.state();
  }

  Eigen::MatrixXd A(2, 2);
  A << 1, dt, 0, 1 - damping * dt / inertia;
  Eigen::MatrixXd B(2, 1);
  B << 0, dt / inertia;
  Eigen::VectorXd x(2);
  x << state.yaw, state.vyaw;
  Eigen::VectorXd u(1);
  u << torque_set;

  gimbal.send(true, false, 0, 0, torque_set, 0, 0, 0);

  int cnt = 0;
  while (!exiter.exit()) {
    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
    data["yaw_measured"] = state.yaw;
    data["vyaw_measured"] = state.vyaw;
    data["yaw_predicted"] = x(0);
    data["vyaw_predicted"] = x(1);
    plotter.plot(data);

    if (cnt < 10) {
      x(1) = state.vyaw;
      cnt++;
    }

    state = gimbal.state();
    x = A * x + B * u;
    x(0) = tools::limit_rad(x(0));

    std::this_thread::sleep_for(10ms);
  }

  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}