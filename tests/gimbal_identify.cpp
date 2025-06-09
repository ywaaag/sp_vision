#include <Eigen/Dense>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

constexpr double DT = 1e-2;
constexpr double J = 19e-2;
constexpr double DAMPING = 0.0;
constexpr double TORQUE_SET = 1.2;

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

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
  A << 1, DT, 0, 1 - DAMPING * DT / J;
  Eigen::MatrixXd B(2, 1);
  B << 0, DT / J;
  Eigen::VectorXd x(2);
  x << state.yaw, state.vyaw;
  Eigen::VectorXd u(1);
  u << TORQUE_SET;

  gimbal.send(true, false, 0, 0, TORQUE_SET, 0, 0, 0);

  while (!exiter.exit()) {
    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
    data["yaw_measured"] = state.yaw;
    data["vyaw_measured"] = state.vyaw;
    data["yaw_predicted"] = x(0);
    data["vyaw_predicted"] = x(1);
    plotter.plot(data);

    state = gimbal.state();
    x = A * x + B * u;
    x(0) = tools::limit_rad(x(0));

    std::this_thread::sleep_for(10ms);
  }

  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}