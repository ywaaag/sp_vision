#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
// #include "io/cboard.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
    "{help h usage ? |      | 输出命令行参数说明}"
    "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char *argv[]) {
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  // 使用串口云台通信替代基于 CAN 的 CBoard
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::GimbalMode::IDLE;
  auto last_mode = io::GimbalMode::IDLE;

  while (!exiter.exit()) {
    const auto loop_start = steady_clock::now();
    camera.read(img, t);
    q = gimbal.q(t - 15ms);
    mode = gimbal.mode();

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img);

    auto targets = tracker.track(armors, t);

    // 调试阶段：使用固定弹速 20 m/s（而非从下位机读取）
    auto command = aimer.aim(targets, t, 20.0);
    // 复用 io::Command 的 horizon_distance
    // 字段，按现有多线程实现的方式计算水平距离
    command.horizon_distance =
        targets.empty() ? 0.0
                        : std::sqrt(tools::square(targets.front().ekf_x()[0]) +
                                    tools::square(targets.front().ekf_x()[2]));

    // 通过串口发送指令（复用已有 Command 接口，内部会按 32B
    // 协议打包并带上时间戳）
    constexpr double kRad2Deg = 180.0 / M_PI;
    const double yaw_deg = command.yaw * kRad2Deg;
    const double pitch_deg = command.pitch * kRad2Deg;

    fmt::print("[Gimbal Cmd] control: {}, shoot: {}, yaw: {:.3f} deg, pitch: "
               "{:.3f} deg, horizon_distance: {:.3f}\n",
               static_cast<int>(command.control),
               static_cast<int>(command.shoot), yaw_deg, pitch_deg,
               command.horizon_distance);

    gimbal.send(command);
    const auto loop_end = steady_clock::now();
    fmt::print("[Loop] elapsed: {} ms\n",
               duration_cast<milliseconds>(loop_end - loop_start).count());
  }

  return 0;
}