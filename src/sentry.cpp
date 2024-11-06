#include <fmt/core.h>

#include <chrono>
#include <future>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim_sentry/aimer.hpp"
#include "tasks/auto_aim_sentry/detector.hpp"
#include "tasks/auto_aim_sentry/solver.hpp"
#include "tasks/auto_aim_sentry/tracker.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{can c          | can0 | can端口名称     }"
  "{debug d        |      | 输出调试画面和信息 }"
  "{@config-path   | configs/usbcamera.yaml | 位置参数，yaml配置文件路径 }";

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
  auto debug = cli.has("debug");
  auto can = cli.get<std::string>("can");
  auto config_path = cli.get<std::string>(0);

  io::CBoard cboard(can);
  io::Camera camera(config_path);
  io::USBCamera usbcam1("video0", config_path);
  io::USBCamera usbcam2("video2", config_path);
  io::USBCamera usbcam3("video4", config_path);

  auto_aim::Detector detector(config_path, debug);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  omniperception::Decider decider(config_path);

  cv::Mat img;
  cv::Mat usb_img1, usb_img2, usb_img3;
  int state;
  const int mode = 1;
  Eigen::Vector2d delta_angle;
  Eigen::Vector3d gimbal_pos;
  std::string send_msg;
  std::string armor_omit = "0,";

  std::chrono::steady_clock::time_point timestamp;

  std::chrono::steady_clock::time_point perception_start, detect_begin, detect_end, perception_end,
    record_start;
  int i = 0;
  for (int frame_count = 0; !exiter.exit(); frame_count++) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
    // recorder.record(img, q, timestamp);

    /// 自瞄核心逻辑

    solver.set_R_gimbal2world(q);

    gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = detector.detect(img, frame_count);

    decider.armor_filter(armors, armor_omit);

    decider.set_priority(armors, mode);

    auto targets = tracker.track(armors, timestamp);

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑

    if (tracker.state() == "lost") {
      perception_start = std::chrono::steady_clock::now();
      usb_img1 = usbcam1.read();
      record_start = std::chrono::steady_clock::now();
      detect_begin = std::chrono::steady_clock::now();
      auto armors_left = detector.detect(usb_img1, frame_count);
      detect_end = std::chrono::steady_clock::now();
      auto left_empty = decider.armor_filter(armors_left, armor_omit);
      if (!left_empty) {
        // left_recorder.record(usb_img1, timestamp);
        delta_angle = decider.delta_angle(armors_left, usbcam1.device_name);
        tools::logger()->debug(
          "delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}", delta_angle[0],
          delta_angle[1], armors_left.size(), auto_aim::ARMOR_NAMES[armors_left.front().name]);
        command = {
          true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
          tools::limit_rad(delta_angle[1] / 57.3)};
      } else {
        usb_img2 = usbcam2.read();
        auto armors_right = detector.detect(usb_img2, frame_count);
        auto right_empty = decider.armor_filter(armors_right, armor_omit);
        if (!right_empty) {
          // right_recorder.record(usb_img2, timestamp);
          delta_angle = decider.delta_angle(armors_right, usbcam2.device_name);
          tools::logger()->debug(
            "delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}", delta_angle[0],
            delta_angle[1], armors_right.size(), auto_aim::ARMOR_NAMES[armors_right.front().name]);
          command = {
            true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
            tools::limit_rad(delta_angle[1] / 57.3)};
        } else {
          usb_img3 = usbcam3.read();
          auto armors_back = detector.detect(usb_img3, frame_count);
          auto back_empty = decider.armor_filter(armors_back, armor_omit);
          if (!back_empty) {
            // back_recorder.record(usb_img3, timestamp);
            delta_angle = decider.delta_angle(armors_back, usbcam3.device_name);
            tools::logger()->debug(
              "delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}", delta_angle[0],
              delta_angle[1], armors_back.size(), auto_aim::ARMOR_NAMES[armors_back.front().name]);
            command = {
              true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
              tools::limit_rad(delta_angle[1] / 57.3)};
          }
        }
      }
      perception_end = std::chrono::steady_clock::now();
    } else {
      command = aimer.aim(targets, timestamp, cboard.bullet_speed);
    }

    if (command.control && tracker.state() == "tracking") command.shoot = true;
    cboard.send(command);
  }

  return 0;
}