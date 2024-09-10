#include <fmt/core.h>

#include <chrono>
#include <future>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim_sentry/aimer.hpp"
#include "tasks/auto_aim_sentry/detector.hpp"
#include "tasks/auto_aim_sentry/solver.hpp"
#include "tasks/auto_aim_sentry/tracker.hpp"
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

  auto_aim::Detector detector(config_path, debug);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img;
  int state;
  const int mode = 1;

  std::string send_msg;

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

    auto armors = detector.detect(img, frame_count);

    auto targets = tracker.track(armors, timestamp);

    /// 调试输出
    if (!debug) continue;

    tools::logger()->info(
      "detect time:{:.2f}ms", abs(tools::delta_time(detect_begin, detect_end) * 1e3));
    tools::logger()->info(
      "perception time:{:.2f}ms", abs(tools::delta_time(perception_start, perception_end) * 1e3));
    tools::logger()->info(
      "record time:{:.2f}ms", abs(tools::delta_time(detect_begin, record_start) * 1e3));

    tools::draw_text(
      img, fmt::format("[{}] [{}]", frame_count, tracker.state()), {10, 30}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
    }

    if (tracker.state() != "lost") {
      auto target = targets.front();
      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["a"] = x[5] * 57.3;
      data["w"] = x[6];
      data["r"] = x[7];
      data["l"] = x[8];
      data["h"] = x[9];
      data["last_id"] = target.last_id;
    }

    // 云台响应情况
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    plotter.plot(data);
    for (auto & armor : armors) {
      tools::draw_points(img, armor.points);
      solver.solve(armor);
      auto xyz = armor.xyz_in_world;
      auto ypr = armor.ypr_in_world * 57.3;

      tools::draw_text(
        img, fmt::format("x{:.2f} y{:.2f} z{:.2f}", xyz[0], xyz[1], xyz[2]), {30, 60});
      tools::draw_text(
        img, fmt::format("r{:.1f} p{:.1f} y{:.1f}", ypr[0], ypr[1], ypr[2]), {30, 100});
    }
    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);

    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  return 0;
}