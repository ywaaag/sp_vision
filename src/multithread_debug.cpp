#include <fmt/core.h>

#include <chrono>
#include <future>
#include <memory>  // 添加头文件
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim_sentry/aimer.hpp"
#include "tasks/auto_aim_sentry/solver.hpp"
#include "tasks/auto_aim_sentry/tracker.hpp"
#include "tasks/auto_aim_sentry/yolov8.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_pool.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

static tools::ThreadPool tp(4);

struct DetectionResult
{
  std::shared_ptr<std::list<auto_aim::Armor>> armors;  // 使用 shared_ptr 管理 Armor 列表
  std::chrono::steady_clock::time_point timestamp;
  double delta_yaw;
  double delta_pitch;
};

// 全局或局部：用于存放 4 个 USBCamera 结果的队列
static tools::ThreadSafeQueue<DetectionResult> detection_queue(10);

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  //io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  io::USBCamera usbcam1("video0", config_path);
  io::USBCamera usbcam2("video2", config_path);
  io::USBCamera usbcam3("video4", config_path);
  io::USBCamera usbcam4("video6", config_path);

  auto_aim::YOLOV8 yolov8(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  omniperception::Decider decider(config_path);

  // 并行任务，读取并推理 USB 相机图像
  auto parallel_infer = [&](io::USBCamera & cam, const std::string & config_path) {
    auto_aim::YOLOV8 yolov8_parallel(config_path, false);
    while (!exiter.exit()) {
      cv::Mat usb_img;
      std::chrono::steady_clock::time_point ts;
      cam.read(usb_img, ts);

      auto results = yolov8_parallel.detect(usb_img);
      auto delta_angle = decider.delta_angle(results, cam.device_name);

      DetectionResult dr;
      dr.armors =
        std::make_shared<std::list<auto_aim::Armor>>(results);  // 使用 shared_ptr 管理 Armor 列表
      dr.timestamp = ts;
      dr.delta_yaw = delta_angle[0];
      dr.delta_pitch = delta_angle[1];
      detection_queue.push(dr);  // 推入线程安全队列
    }
  };

  // 将 4 个相机推理任务加入线程池
  tp.add_task([&] { parallel_infer(usbcam1, config_path); });
  tp.add_task([&] { parallel_infer(usbcam2, config_path); });
  tp.add_task([&] { parallel_infer(usbcam3, config_path); });
  tp.add_task([&] { parallel_infer(usbcam4, config_path); });

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    // Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);

    /// 自瞄核心逻辑
    // solver.set_R_gimbal2world(q);

    // Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolov8.detect(img);

    decider.armor_filter(armors);

    decider.set_priority(armors);

    auto targets = tracker.track(armors, timestamp);

    // 将队列中的对象全部弹出放入 vector
    std::vector<DetectionResult> all_results;
    while (!detection_queue.empty()) {
      DetectionResult dr;
      detection_queue.pop(dr);
      all_results.push_back(dr);
    }

    // 对 all_results 进行排序，比如按时间戳升序
    std::sort(all_results.begin(), all_results.end(), [](const auto & a, const auto & b) {
      return a.timestamp < b.timestamp;
    });

    tools::logger()->debug("all_results size:{}", all_results.size());
    /*
    if()
    */

    // io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
    // if (tracker.state() == "lost")
    //   command = decider.decide(yolov8, gimbal_pos, usbcam1, usbcam2, usbcam3, usbcam4);
    // else
    //   command = aimer.aim(targets, timestamp, cboard.bullet_speed);

    // if (
    //   command.control && aimer.debug_aim_point.valid &&
    //   std::abs(last_command.yaw - command.yaw) * 57.3 < 2 &&
    //   std::abs(gimbal_pos[0] - last_command.yaw) * 57.3 < 1.5) {  //应该减去上一次command的yaw值
    //   tools::logger()->debug("#####shoot#####");
    //   command.shoot = true;
    // }

    // if (command.control) last_command = command;

    // cboard.send(command);
  }

  return 0;
}