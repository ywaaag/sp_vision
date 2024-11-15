#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
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
#include "tools/unscented_kalman_filter.hpp"

using namespace std::chrono;

#define M_PI       3.14159265358979323846

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard5.yaml | 位置参数，yaml配置文件路径 }";

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
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  /// 调试输出
  nlohmann::json data;

  double max_x=0, min_x=0, max_y=0, min_y=0;
  cv::Point2f center;

  Eigen::MatrixXd points_initial(4, 3);
  Eigen::VectorXd x_init(6);
  Eigen::MatrixXd P_init;
  Eigen::MatrixXd z(4, 3);
  double dt;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(12, 12) * 0.1;
  tools::UnscentedKalmanFilter UKF(x_init, P_init, dt);
  bool UKFinitialized = false;


  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    // recorder.record(img, q, t);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);
    std::list<auto_aim::Armor> armors = yolov8.detect(img);
    if(!armors.empty()) solver.solve(armors.front());
    if(!UKFinitialized && !armors.empty()) {
      UKFinitialized = true;
      auto_aim::Armor armor = armors.front();
      x_init << armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2],
                0.753982, 0, 0.8*M_PI;
      P_init = Eigen::MatrixXd::Identity(6, 6);
      dt = 0.1;
      UKF.x = x_init;
      UKF.P = P_init;
      UKF.dt = dt;
      for(int i=0; i<4; ++i) {
        points_initial.row(i) << armor.object_points_world[i].x,
                                armor.object_points_world[i].y,
                                armor.object_points_world[i].z;
      }
    } else if(UKFinitialized && armors.empty()) {
      UKFinitialized = false;
    } else if(UKFinitialized && !armors.empty()) {
      auto_aim::Armor armor = armors.front();
      for(int i=0; i<4; ++i) {
        z.row(i) << armor.object_points_world[i].x,
                    armor.object_points_world[i].y,
                    armor.object_points_world[i].z;
      }
      UKF.predict();
      UKF.update(z, R, points_initial);
    }

    // 传出装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
    }

    // 计算前哨站旋转中心
    max_x = std::max(armors.front().xyz_in_world[0], max_x);
    max_y = std::max(armors.front().xyz_in_world[1], max_y);
    min_x = std::min(armors.front().xyz_in_world[0], min_x);
    min_y = std::min(armors.front().xyz_in_world[1], min_y);
    center.x = max_x/2 + min_x/2;
    center.y = max_y/2 + min_y/2;
    data["center_x"] = center.x;
    data["center_y"] = center.y;
  
    plotter.plot(data);


    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  return 0;
}