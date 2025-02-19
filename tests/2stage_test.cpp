#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim_sentry/yolo11.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@pic_path      |                        | 图片路径}"
  "{config-path c  | configs/newsentry.yaml | yaml配置文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto pic_path = cli.get<std::string>(0);  // 图片路径
  auto config_path = cli.get<std::string>("config-path");
  tools::logger()->debug("pic_path: {}", pic_path);
  tools::Exiter exiter;

  auto_aim::YOLO11 yolo11(config_path, true);

  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    // 读取图片
    cv::Mat img = cv::imread(pic_path);
    if (img.empty()) {
      tools::logger()->error("无法读取图片: {}", pic_path);
      break;
    }
    std::list<auto_aim::Armor> armors;
    armors = yolo11.detect(img);

    // 按 'q' 键退出
    auto key = cv::waitKey(0);
    if (key == 'q') break;
  }
  return 0;
}
