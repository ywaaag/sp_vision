#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{delta-angle a  |          8          | yaw轴delta角}"
  "{circle      c  |         0.2         | delta_angle的切片数}"
  "{signal-mode m  |     triangle_wave   | 发送信号的模式}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

double yaw_cal(double t)
{
  double A = 7;
  double T = 6; // s
  
  return A * std::sin(2 * M_PI * t / T) ; // 31是云台yaw初始角度，单位为度
}

double pitch_cal(double t)
{
  double A = 7;
  double T = 6; // s

  return A * std::sin(2 * M_PI * t / T + M_PI / 2) + 18; // 18是云台pitch初始角度，单位为度
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  auto delta_angle = cli.get<double>("delta-angle");
  auto circle = cli.get<double>("circle");
  auto signal_mode = cli.get<std::string>("signal-mode");
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::CBoard cboard(config_path);

  auto init_yaw = 0;
  double slice = circle * 100;  //切片数=周期*帧率
  auto dyaw = delta_angle / slice;
  double cmd_yaw = init_yaw;
  double error = 0;
  int count = 0;
  io::Command init_command{1, 0, 0, 0};
  cboard.send(init_command);
  std::this_thread::sleep_for(5s);  //等待云台归零

  io::Command command{0};
  io::Command last_command{0};

  double t = 0;
  double dt = 0.010;  // 5ms, 模拟200fps

  while (!exiter.exit()) {
    nlohmann::json data;
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0);

    if (signal_mode == "triangle_wave") {
      if (count == slice) {
        cmd_yaw = init_yaw;
        command = {1, 0, cmd_yaw / 57.3, 0};
        count = 0;

      } else {
        cmd_yaw += dyaw;
        command = {1, 0, cmd_yaw / 57.3, 0};
        count++;
      }

      cboard.send(command);
      data["cmd_yaw"] = command.yaw * 57.3;
      data["last_cmd_yaw"] = last_command.yaw * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      last_command = command;
      plotter.plot(data);
      std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    }

    else if (signal_mode == "step") {
      if (count == 300) {
        cmd_yaw += delta_angle;
        count = 0;
      }
      command = {1, 0, tools::limit_rad(cmd_yaw / 57.3), 0};
      count++;

      cboard.send(command);
      data["cmd_yaw"] = command.yaw * 57.3;
      data["last_cmd_yaw"] = last_command.yaw * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      last_command = command;
      plotter.plot(data);
      std::this_thread::sleep_for(8ms);  //模拟自瞄100fps
    } 
    
    else if (signal_mode == "circle") {
      std::cout << "t: " << t << std::endl;
      command.yaw = yaw_cal(t) / 57.3;
      command.pitch = pitch_cal(t) / 57.3;
      command.control = 1;
      command.shoot = 0;
      t += dt;
      cboard.send(command);

      data["t"] = t;
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["gimbal_yaw"] = eulers[0] * 57.3;
      data["gimbal_pitch"] = eulers[1] * 57.3;
      plotter.plot(data);
      std::this_thread::sleep_for(9ms);
    }
  }
  return 0;
}