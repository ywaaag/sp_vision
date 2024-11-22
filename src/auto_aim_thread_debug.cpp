#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard5.yaml | 位置参数，yaml配置文件路径 }";

std::mutex mtx; // 用于保护对共享资源的访问

// 处理任务的线程函数
void process_frame(const cv::Mat& img, const std::chrono::steady_clock::time_point& t, 
                   io::CBoard& cboard, auto_aim::YOLOV8& yolov8, 
                   auto_aim::Solver& solver, auto_aim::Tracker& tracker, 
                   auto_aim::Aimer& aimer) {
    // 线程内处理逻辑
    Eigen::Quaterniond q = cboard.imu_at(t - 1ms);
    solver.set_R_gimbal2world(q);

    // 执行目标检测
    auto armors = yolov8.detect(img);

    // 执行目标追踪
    auto targets = tracker.track(armors, t);

    // 执行目标瞄准
    auto command = aimer.aim(targets, t, cboard.bullet_speed);

    // 发送控制命令
    cboard.send(command);

        /// 调试输出

    tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
    }

    if (!targets.empty()) {
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
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;
    }

    // 云台响应情况
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
    }

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
}

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder(100);  //根据实际帧率调整

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLOV8 yolov8(config_path, true);
  auto_aim::Solver solver(config_path);
  auto useless = auto_aim::Target(auto_aim::ArmorName::base, auto_aim::ArmorType::big, 4);
  auto_aim::Tracker tracker(config_path, solver, useless);
  // auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    // 使用 std::lock_guard 来保证线程安全
    std::lock_guard<std::mutex> lock(mtx);

    // 创建一个新的线程处理目标检测、追踪、瞄准、命令发送
    std::thread processing_thread(process_frame, std::cref(img), t, std::ref(cboard),
                                  std::ref(yolov8), std::ref(solver), 
                                  std::ref(tracker), std::ref(aimer));

    // 启动处理线程后立即继续读取下一帧
    processing_thread.detach(); // 使线程在后台执行
    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  return 0;
}