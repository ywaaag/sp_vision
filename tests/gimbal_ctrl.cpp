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
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

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

  while (!exiter.exit()) {
    auto state = gimbal.state();
    auto yaw = state.yaw;
    auto vyaw = state.vyaw;
    auto pitch = state.pitch;
    auto vpitch = state.vpitch;

    nlohmann::json data;
    data["yaw"] = yaw;
    data["vyaw"] = vyaw;
    data["pitch"] = pitch;
    data["vpitch"] = vpitch;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
    plotter.plot(data);

    std::this_thread::sleep_for(10ms);
  }

  return 0;
}