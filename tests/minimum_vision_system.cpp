#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>
#include <iostream>

#include "io/camera.hpp"
// 移除对 io/dm_imu/dm_imu.hpp 的引用
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>("@config-path");

  tools::Exiter exiter;
  tools::Plotter plotter;
  io::Camera camera(config_path);
  auto yaml = tools::load(config_path);
  bool visualize = true;
  if (yaml["visualize"]) visualize = yaml["visualize"].as<bool>();
  // 移除对 io::DM_IMU dm_imu 的实例化

  auto_aim::multithread::MultiThreadDetector detector(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!exiter.exit()) {
      camera.read(img, t);
      detector.push(img, t);
    }
  });

  auto last_t = std::chrono::steady_clock::now();
  nlohmann::json data;

  // 定义固定的弹速和四元数
  const double fixed_bullet_speed = 22.0; // 设置为固定的子弹速度
  Eigen::Quaterniond fixed_q(1, 0, 0, 0); // 设置固定的单位四元数（无旋转）

  while (!exiter.exit()) {
    auto [img, armors, t] = detector.debug_pop();
    
    // 直接使用固定的四元数，不再从IMU读取
    Eigen::Quaterniond q = fixed_q;

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto targets = tracker.track(armors, t);

    // 直接使用固定的子弹速度，不再从cboard读取
    auto command = aimer.aim(targets, t, fixed_bullet_speed);

    shooter.shoot(command, aimer, targets, gimbal_pos);

    auto dt = tools::delta_time(t, last_t);
    last_t = t;

    data["dt"] = dt;
    data["fps"] = 1 / dt;
    plotter.plot(data);
    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      auto min_x = 1e10;
      auto & armor = armors.front();
      for (auto & a : armors) {
        if (a.center.x < min_x) {
          min_x = a.center.x;
          armor = a;
        }
      }
      solver.solve(armor);
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;

      std::cout << "Armor Position: (" 
                << armor.xyz_in_world[0] << ", "
                << armor.xyz_in_world[1] << ", "
                << armor.xyz_in_world[2] << ") "
                << "Yaw: " << armor.ypr_in_world[0] * 57.3
                << ", Pitch: " << armor.ypr_in_world[1] * 57.3
                << ", Roll: " << armor.ypr_in_world[2] * 57.3
                << std::endl;
    }

    if (!targets.empty()) {
      auto target = targets.front();
      tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});
    }
    if (visualize) {
      cv::resize(img, img, {}, 0.5, 0.5);
      cv::imshow("reprojection", img);
      auto key = cv::waitKey(1);
      if (key == 'q') break;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  detect_thread.join();

  return 0;
}