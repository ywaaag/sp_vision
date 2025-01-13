#include "perceptron.hpp"

#include <chrono>
#include <thread>

#include "tasks/auto_aim_sentry/yolov8.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

namespace omniperception
{
Perceptron::Perceptron(
  io::USBCamera & usbcam1, io::USBCamera & usbcam2, io::USBCamera & usbcam3,
  io::USBCamera & usbcam4, const std::string & config_path)
: thread_pool_(4), detection_queue_(10), decider_(config_path), stop_flag_(false)
{
  // 并行任务，读取并推理 USB 相机图像
  auto parallel_infer = [&](io::USBCamera & cam, const std::string & config_path) {
    auto_aim::YOLOV8 yolov8_parallel(config_path, false);
    while (true) {
      cv::Mat usb_img;
      std::chrono::steady_clock::time_point ts;

      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_flag_) break;  // 检查是否需要退出
      }

      cam.read(usb_img, ts);

      auto armors = yolov8_parallel.detect(usb_img);
      if (!armors.empty()) {
        auto delta_angle = decider_.delta_angle(armors, cam.device_name);

        DetectionResult dr;
        dr.armors = std::move(armors);
        dr.timestamp = ts;
        dr.delta_yaw = delta_angle[0];
        dr.delta_pitch = delta_angle[1];

        detection_queue_.push(dr);  // 推入线程安全队列
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  };

  // 将 4 个相机推理任务加入线程池
  thread_pool_.add_task([&] { parallel_infer(usbcam1, config_path); });
  thread_pool_.add_task([&] { parallel_infer(usbcam2, config_path); });
  thread_pool_.add_task([&] { parallel_infer(usbcam3, config_path); });
  thread_pool_.add_task([&] { parallel_infer(usbcam4, config_path); });

  tools::logger()->info("Perceptron initialized.");
}

Perceptron::~Perceptron()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_flag_ = true;  // 设置退出标志
  }
  condition_.notify_all();  // 唤醒所有等待的线程

  // 等待线程池中的所有线程完成
  thread_pool_.wait_for_tasks();
}

const tools::ThreadSafeQueue<DetectionResult> & Perceptron::get_detection_queue() const
{
  return detection_queue_;
}

}  // namespace omniperception