#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"  // ← 新增：串口云台
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  auto yaml = tools::load(config_path);
  tools::Exiter exiter;

  // ====== 初始化硬件 ======
  io::Camera camera(config_path);

  // 使用串口云台（从 YAML 配置读取是否启用）
  std::unique_ptr<io::Gimbal> gimbal = nullptr;
  if (yaml["gimbal_use_serial"] && yaml["gimbal_use_serial"].as<bool>()) {
    gimbal = std::make_unique<io::Gimbal>(config_path);
  } else {
    tools::logger()->error("gimbal_use_serial is not enabled in config!");
    return -1;
  }

  // ====== 初始化算法模块 ======
  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  // ====== 主循环 ======
  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto last_t = t;

  tools::logger()->info("Starting main loop (no visualization, console output only)...");

  while (!exiter.exit()) {
    // 1. 读取图像
    camera.read(img, t);
    if (img.empty()) {
      std::this_thread::sleep_for(milliseconds(1));
      continue;
    }

    // 2. 从云台获取四元数（关键修改）
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    if (gimbal) {
      auto gimbal_q = gimbal->q(t);
      if (gimbal_q.norm() > 1e-3) { // 有效四元数
        q = gimbal_q;
      }
    }
    solver.set_R_gimbal2world(q);

    // 3. 检测与追踪
    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, t);

    // 4. 打印检测到的装甲板参数（核心需求）
    if (!armors.empty()) {
      tools::logger()->info("Detected {} armors:", armors.size());
      for ( auto& armor : armors) {
        // 解算位姿（必须调用！）
        solver.solve(armor);

        // 世界坐标 XYZ
        Eigen::Vector3d xyz = armor.xyz_in_world;
        // 世界姿态 YPR（转为角度）
        Eigen::Vector3d ypr = armor.ypr_in_world * 57.3;

        tools::logger()->info(
          "  Armor {}: Pos=({:.2f}, {:.2f}, {:.2f}) m, "
          "YPR=({:.1f}, {:.1f}, {:.1f}) deg, "
          "Conf={:.2f}",
          armor.name,
          xyz[0], xyz[1], xyz[2],
          ypr[0], ypr[1], ypr[2],
          armor.confidence
        );
      }
    }

    // 5. 控制逻辑（使用固定弹速或从云台读取）
    double bullet_speed = 22.0; // 可从云台扩展
    auto command = aimer.aim(targets, t, bullet_speed);

    // 6. 发送控制指令（通过云台串口）
    if (gimbal) {
      gimbal->send_gimbal_cmd(
        command.shoot,
        command.pitch,
        command.yaw,
        command.horizon_distance,
        0, 0 // reserved
      );
    }

    // 7. 帧率控制（可选）
    auto dt = tools::delta_time(t, last_t);
    last_t = t;
    if (dt > 0) {
      tools::logger()->debug("FPS: {:.1f}", 1.0 / dt);
    }

    // 按 'q' 退出（需启用 waitKey，但无 imshow）
    if (cv::waitKey(1) == 'q') {
      break;
    }
  }

  tools::logger()->info("Program exited.");
  return 0;
}