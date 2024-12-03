#include <fmt/core.h>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/auto_shoot_aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

// 引入线程池
#include "thread_pool.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard5.yaml | 位置参数，yaml配置文件路径 }";

std::mutex mtx; // 用于保护对共享资源的访问

// 处理任务的线程函数
void process_frame(cv::Mat& img, const std::chrono::steady_clock::time_point& t, 
                   io::CBoard& cboard, auto_aim::YOLOV8& yolov8, 
                   auto_aim::Solver& solver, auto_aim::Tracker& tracker, 
                   auto_aim::Aimer& aimer, tools::Plotter& plotter) {
    Eigen::Quaterniond q = cboard.imu_at(t - 1ms);
    solver.set_R_gimbal2world(q);

    // 执行目标检测
    auto armors = yolov8.detect(img);

    // 执行目标追踪
    auto targets = tracker.track(armors, t);

    // 执行目标瞄准
    auto command = aimer.aim(targets, armors, t, cboard.bullet_speed);

    // 发送控制命令
    cboard.send(command);


    nlohmann::json data;
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto& armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
    }

    if (!targets.empty()) {
      auto target = targets.front();
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d& xyza : armor_xyza_list) {
        auto image_points = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points = solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});

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

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
    }

    plotter.plot(data);
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("reprojection", img);
}

int main(int argc, char * argv[]) {
    tools::Exiter exiter;
    tools::Plotter plotter;
    tools::Recorder recorder(100);

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
    std::chrono::steady_clock::time_point last_t = std::chrono::steady_clock::now();

    // 测试线程数与帧率的关系
    ThreadPool thread_pool(12);

    while (!exiter.exit()) {
        camera.read(img, t);
        auto dt = tools::delta_time(t, last_t);
        last_t = t;
        // tools::logger()->info("{:.2f} fps", 1 / dt);
        tools::draw_text(img, fmt::format("{:.2f} fps", 1/dt), {10, 60}, {255, 255, 255});
        nlohmann::json data;
        data["fps"] = 1/dt;
        plotter.plot(data);

        // 将处理任务提交到线程池
        thread_pool.enqueue([&] {
            std::lock_guard<std::mutex> lock(mtx);
            process_frame(img, t, cboard, yolov8, solver, tracker, aimer, plotter);
        });

        auto key = cv::waitKey(1);
        if (key == 'q') break;
    }

    return 0;
}
