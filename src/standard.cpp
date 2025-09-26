#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"  
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/timer.hpp"
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
  // 计时：初始化 I/O 设备耗时
  tools::ScopedTimer init_timer("1. Init Hardware & Modules"); 
  
  io::Camera camera(config_path);

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

  // 初始化计时器在退出作用域时自动停止
  // init_timer 计时结束

  // ====== 主循环 ======
  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  auto last_t = t;

  tools::logger()->info("Starting main loop (no visualization, console output only)...");
  
  // 移除错误的全局计时器，改为在循环内使用 ScopedTimer

  while (!exiter.exit()) {
    // 【关键修改】：总帧计时器，测量整个循环耗时
    tools::ScopedTimer total_frame_timer("Total Frame Time"); 

    // 1. 读取图像
    {
      tools::ScopedTimer timer("2. Read Image (I/O)"); // 计时：图像读取耗时
      camera.read(img, t);
    } // I/O 计时结束

    if (img.empty()) {
      std::this_thread::sleep_for(milliseconds(1));
      continue;
    }

    // 2. 从云台获取四元数（关键修改）
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    if (gimbal) {
      // 计时：I/O 耗时
      tools::ScopedTimer timer("2.1. Get Gimbal Q");
      auto gimbal_q = gimbal->q(t);
      if (gimbal_q.norm() > 1e-3) {
        q = gimbal_q;
      }
    }
    solver.set_R_gimbal2world(q);

    // 3. 检测与追踪
    decltype(detector.detect(img)) armors;
    {
      tools::ScopedTimer timer("3.1. Detect"); // 计时：YOLO/装甲板检测
      armors = detector.detect(img);
    }

    decltype(tracker.track(armors, t)) targets;
    {
      tools::ScopedTimer timer("3.2. Track"); // 计时：追踪算法
      targets = tracker.track(armors, t);
    }

    // 4. 打印检测到的装甲板参数
    if (!armors.empty()) {
      tools::ScopedTimer timer("4. Solve PnP and Process"); // 计时：解算与处理
      tools::logger()->info("Detected {} armors:", armors.size());
      for (auto & armor : armors) {
        // 解算位姿（必须调用！）
        solver.solve(armor);

        // 世界坐标 XYZ
        Eigen::Vector3d xyz = armor.xyz_in_world;
        // 世界姿态 YPR（转为角度）
        Eigen::Vector3d ypr = armor.ypr_in_world * 57.3;

        // 日志打印操作也会占用时间，但通常很少
      }
    }

    // 5. 控制逻辑
    double bullet_speed = 22.0;  // 可从云台扩展
    auto command = aimer.aim(targets, t, bullet_speed);

    // 6. 发送控制指令（通过云台串口）
    if (gimbal) {
      tools::ScopedTimer timer("5. Send Gimbal Cmd"); // 计时：I/O 耗时
      gimbal->send_gimbal_cmd(
        command.shoot, command.pitch, command.yaw, command.horizon_distance, 0, 0
      );
    }

    // 7. 帧率计算（基于相机时间戳 t）
    auto dt = tools::delta_time(t, last_t);
    last_t = t;
    if (dt > 0) {
      tools::logger()->debug("FPS (by camera dt): {:.1f}", 1.0 / dt);
    }

    // 按 'q' 退出
    if (cv::waitKey(1) == 'q') {
      break;
    }
    
  } // total_frame_timer 计时结束，打印准确的单帧耗时

  tools::logger()->info("Program exited.");
  return 0;
}