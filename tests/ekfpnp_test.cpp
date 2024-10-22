#include "tasks/auto_aim_sentry/ekfpnp.hpp"

#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim_sentry/solver.hpp"
#include "tasks/auto_aim_sentry/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/ekf.yaml    | yaml配置文件的路径}"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{@input-path    |                        | avi和txt文件的路径}";

constexpr double LIGHTBAR_LENGTH = 56e-3;     // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  auto_aim::YOLOV8 detector(config_path);
  auto_aim::EKFPnP ekfpnp(config_path);
  auto_aim::Solver solver(config_path);

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  double last_t = -1;

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  int ekf = 0;
  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    /// 自瞄核心逻辑

    solver.set_R_gimbal2world({w, x, y, z});

    auto detector_start = std::chrono::steady_clock::now();
    auto armors = detector.detect(img, frame_count);

    auto solve_start = std::chrono::steady_clock::now();
    if (!armors.empty()) {
      auto armor = armors.front();
      Eigen::Vector3d t_armor2camera;
      Eigen::Matrix3d R_armor2camera;
      if (ekf == 0) {
        Eigen::Quaterniond q_camera2armor;
        Eigen::Vector3d camera_in_xyz;
        solver.solve(armor, q_camera2armor, camera_in_xyz);
        ekfpnp.init_EKFPnP(camera_in_xyz, q_camera2armor, timestamp);
        ++ekf;
      } else {
        ekfpnp.iterate_EKFPnP(
          t_armor2camera, R_armor2camera, timestamp, SMALL_ARMOR_POINTS, armor.points);
        solver.solve(armor);
      }
    }
    auto solve_end = std::chrono::steady_clock::now();
    tools::draw_text(img, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

    tools::logger()->info(
      "[{}] detector: {:.1f}ms, solver: {:.1f}ms", frame_count,
      tools::delta_time(solve_start, detector_start) * 1e3,
      tools::delta_time(solve_end, solve_start) * 1e3);

    // nlohmann::json data;

    // // 装甲板原始观测数据
    // data["armor_num"] = armors.size();
    // if (!armors.empty()) {
    //   const auto & armor = armors.front();
    //   data["armor_x"] = armor.xyz_in_world[0];
    //   data["armor_y"] = armor.xyz_in_world[1];
    //   data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
    //   data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    //   data["armor_center_x"] = armor.center_norm.x;
    //   data["armor_center_y"] = armor.center_norm.y;
    // }

    // plotter.plot(data);

    // cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    // cv::imshow("reprojection", img);
    auto key = cv::waitKey(10);
    if (key == 'q') break;
  }
  ekfpnp.deinit_EKFPnP();

  return 0;
}