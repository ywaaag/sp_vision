#include "io/camera.hpp"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include "tasks/auto_aim/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml | yaml配置文件路径 }"
  "{d display      |                     | 显示视频流       }";

// 任务处理函数，将图像帧传递给工作线程
void process_frame(cv::Mat& img, const std::chrono::steady_clock::time_point& timestamp,
                   auto_aim::YOLOV8& yolov8, tools::Plotter& plotter) {
    // 目标检测
    auto armors = yolov8.detect(img);

    // 数据记录
    nlohmann::json data;
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
        const auto& armor = armors.front();
        data["armor_center_x"] = armor.center_norm.x;
        data["armor_center_y"] = armor.center_norm.y;
    }

    tools::logger()->info("Detected {} armors", armors.size());
    plotter.plot(data);

    // 在调试模式下显示图像
    cv::imshow("img", img);
}

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    tools::Exiter exiter;
    tools::Plotter plotter;

    auto config_path = cli.get<std::string>("config-path");
    auto display = cli.has("display");
    io::Camera camera(config_path);
    auto_aim::YOLOV8 yolov8(config_path, true);

    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    auto last_stamp = std::chrono::steady_clock::now();
    std::mutex mtx;  // 保护共享资源

    while (!exiter.exit()) {
        // 从相机读取图像
        camera.read(img, timestamp);

        auto dt = tools::delta_time(timestamp, last_stamp);
        last_stamp = timestamp;

        // 使用 std::lock_guard 确保线程安全
        std::lock_guard<std::mutex> lock(mtx);

        // 创建并启动一个新线程来处理当前帧
        std::thread processing_thread(process_frame, std::ref(img), timestamp, std::ref(yolov8), std::ref(plotter));

        // 启动线程后立即继续读取下一帧
        processing_thread.detach();  // 让线程在后台执行

        tools::logger()->info("{:.2f} fps", 1 / dt);

        // 如果不需要显示图像，则跳过
        if (!display) continue;

        // 允许用户按 'q' 键退出
        if (cv::waitKey(1) == 'q') break;
    }

    return 0;
}
