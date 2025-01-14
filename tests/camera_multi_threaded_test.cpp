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

#include "../src/thread_pool.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard5.yaml | 位置参数，yaml配置文件路径 }";

std::mutex mtx; // 用于保护对共享资源的访问

// 处理任务的线程函数
void process_frame(cv::Mat& img, const std::chrono::steady_clock::time_point& t, 
                    auto_aim::YOLOV8& yolov8, tools::Plotter& plotter, int i) {

    // 执行目标检测
    auto armors = yolov8.detect(img);
    // cv::resize(img, img, {}, 0.5, 0.5);
    // cv::imshow("reprojection", img);
}

int main(int argc, char * argv[]) {
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

    std::vector<auto_aim::YOLOV8> yolov8s;
    int num_yolov8 = 12;
    std::vector<bool> yolo_used(num_yolov8, false);
    for(int i=0; i<num_yolov8; i++) {
      yolov8s.push_back(auto_aim::YOLOV8(config_path, true));
    }
        
    // 测试线程数与帧率的关系
    ThreadPool thread_pool(12);
    int count = 0;

    while (!exiter.exit()) {
        camera.read(img, t);
        auto dt = tools::delta_time(t, last_t);
        last_t = t;
        // tools::logger()->info("time between{}&{} is{:.6f}", count, count+1, dt);
        tools::draw_text(img, fmt::format("{:.2f} fps", 1/dt), {10, 60}, {255, 255, 255});
        nlohmann::json data;
        data["fps"] = 1/dt;

        // 将处理任务提交到线程池
        std::mutex yolo_mutex;

        // std::chrono::steady_clock::time_point enqueue_t = std::chrono::steady_clock::now();
        // auto before_thread = tools::delta_time(enqueue_t, t);
        // tools::logger()->info("{} time used before add thread is {:.6f}", count, before_thread);
        
        ++count;
        thread_pool.enqueue([&] {
          auto_aim::YOLOV8* yolo = nullptr;
          for (int i = 0; i < num_yolov8; i++) {
            if (!yolo_used[i]) {
              yolo_used[i] = true;
              yolo = &yolov8s[i];
              break;
            }
          }
          if (yolo) {
            auto img_copy = img.clone();
            // auto img_copy = std::move(img);

            process_frame(img_copy, t, *yolo, plotter, count);
            for (int i = 0; i < num_yolov8; i++) {
              if (yolo == &yolov8s[i]) {
                yolo_used[i] = false;
                break;
              }
            }
          }
        });
        // std::chrono::steady_clock::time_point after_enqueue_t = std::chrono::steady_clock::now();
        // auto after_thread = tools::delta_time(after_enqueue_t, enqueue_t);
        // tools::logger()->info("{} time used for thread add {:.6f}", count, after_thread);
        plotter.plot(data);

        auto key = cv::waitKey(1);
        // std::chrono::steady_clock::time_point end_t = std::chrono::steady_clock::now();
        // auto end_ttt = tools::delta_time(end_t, after_enqueue_t);
        // tools::logger()->info("timeused at end {:.2f}", end_t);
        if (key == 'q') break;
    }

    return 0;
}
