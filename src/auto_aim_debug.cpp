#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();

    while (!exiter.exit()) {
      auto target = target_queue.front();
      auto gs = gimbal.state();
      auto plan = planner.plan(target, gs);

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.vyaw, plan.yaw_torque, plan.pitch, plan.vpitch,
        plan.pitch_torque);

      nlohmann::json data;

      // 云台响应情况
      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_vyaw"] = gs.vyaw;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_vpitch"] = gs.vpitch;
      data["bullet_speed"] = gs.bullet_speed;

      data["ref_yaw"] = plan.yaw;
      data["ref_vyaw"] = plan.vyaw;
      data["ref_pitch"] = plan.pitch;
      data["ref_vpitch"] = plan.vpitch;
      data["torque_yaw"] = plan.yaw_torque;
      data["torque_pitch"] = plan.pitch_torque;

      data["control"] = plan.control ? 1 : 0;

      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      plotter.plot(data);
      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    auto q = gimbal.q(t);

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);
  }

  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}