#include <fmt/core.h>

#include <chrono>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <variant>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo11.hpp"
#include "tasks/auto_aim/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_pool.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard5.yaml | 位置参数，yaml配置文件路径 }";

std::mutex mtx;  // 用于保护对共享资源的访问

// 处理任务的线程函数
void process_frame(
  cv::Mat & img, const std::chrono::steady_clock::time_point & t, auto_aim::YOLOV8 & yolo,
  tools::Plotter & plotter)
{
  // 执行目标检测
  auto armors = yolo.detect(img);
}

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  // tools::Recorder recorder(100);

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Camera camera(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point last_t = std::chrono::steady_clock::now();

  int num_yolo_thread = 6;
  auto yolos = tools::create_yolov8s(config_path, num_yolo_thread, true);
  // auto yolos = tools::create_yolo11s(config_path, num_yolo_thread, true);
  std::vector<bool> yolo_used(num_yolo_thread, false);
  tools::ThreadPool thread_pool(num_yolo_thread);

  while (!exiter.exit()) {
    camera.read(img, t);
    auto dt = tools::delta_time(t, last_t);
    last_t = t;
    tools::draw_text(img, fmt::format("{:.2f} fps", 1 / dt), {10, 60}, {255, 255, 255});
    nlohmann::json data;
    data["fps"] = 1 / dt;

    std::mutex yolo_mutex;
    thread_pool.enqueue([&] {
      auto_aim::YOLOV8 * yolo = nullptr;
      for (int i = 0; i < num_yolo_thread; i++) {
        if (!yolo_used[i]) {
          yolo_used[i] = true;
          yolo = &yolos[i];
          break;
        }
      }
      if (yolo) {
        auto img_copy = img.clone();
        process_frame(img_copy, t, *yolo, plotter);
        for (int i = 0; i < num_yolo_thread; i++) {
          if (yolo == &yolos[i]) {
            yolo_used[i] = false;
            break;
          }
        }
      }
    });
    plotter.plot(data);

    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
