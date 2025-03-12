#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/yolo11.hpp"
#include "tasks/auto_aim/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{@video_path    |                        | avi路径}"
  "{use-yolov8  8  |           false        | avi路径}"
  "{config-path c  | configs/sentry.yaml    | yaml配置文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto video_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");
  auto use_yolov8 = cli.get<bool>("use-yolov8");
  tools::logger()->debug("video_path: {}", video_path);
  tools::Exiter exiter;

  cv::VideoCapture video(video_path);

  auto_aim::YOLO11 yolo11(config_path, true);
  auto_aim::YOLOV8 yolov8(config_path, true);

  std::chrono::steady_clock::time_point timestamp;
  tools::logger()->debug("here");

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    cv::Mat img;
    std::list<auto_aim::Armor> armors;
    video.read(img);
    if (img.empty()) break;

    auto last = std::chrono::steady_clock::now();

    if (use_yolov8)
      armors = yolov8.detect(img);
    else
      armors = yolo11.detect(img);

    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last);
    tools::logger()->info("{:.2f} fps", 1 / dt);

    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  return 0;
}