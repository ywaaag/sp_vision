#include <fmt/core.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                     | 输出命令行参数说明}"
  "{config-path c   | configs/camera.yaml | yaml配置文件路径 }"
  "{output-folder o | assets/outpostDataSetImg  | 输出文件夹路径   }";

void capture_loop(
  const std::string & config_path, const std::string & can, const std::string & output_folder)
{
  io::Camera camera(config_path);
  cv::Mat img;
  cv::Mat img_show;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;
  while (true) {
    camera.read(img, timestamp);

    // 按“s”保存图片和对应四元数，按“q”退出程序
    cv::resize(img, img_show, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("Press s to save, q to quit", img);
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    // 保存图片和四元数
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    cv::imwrite(img_path, img);
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }

  // 离开该作用域时，camera和cboard会自动关闭
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>("config-path");
  auto output_folder = cli.get<std::string>("output-folder");

  // 新建输出文件夹
  std::filesystem::create_directory(output_folder);

  // 主循环，保存图片
  capture_loop(config_path, "can0", output_folder);


  return 0;
}
