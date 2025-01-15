#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <chrono>
#include <list>
#include <memory>

#include "decider.hpp"
#include "detection.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim_sentry/armor.hpp"
#include "tools/thread_pool.hpp"
#include "tools/thread_safe_queue.hpp"

namespace omniperception
{

class Perceptron
{
public:
  Perceptron(
    io::USBCamera * usbcma1, io::USBCamera * usbcam2, io::USBCamera * usbcam3,
    io::USBCamera * usbcam4, const std::string & config_path);

  ~Perceptron();

  tools::ThreadSafeQueue<DetectionResult> get_detection_queue() const;

  void parallel_infer(io::USBCamera * cam, std::shared_ptr<auto_aim::YOLOV8> & yolov8_parallel);

private:
  // tools::ThreadPool thread_pool_;
  std::vector<std::thread> threads_;
  tools::ThreadSafeQueue<DetectionResult> detection_queue_;

  std::shared_ptr<auto_aim::YOLOV8> yolov8_parallel1_;
  std::shared_ptr<auto_aim::YOLOV8> yolov8_parallel2_;
  std::shared_ptr<auto_aim::YOLOV8> yolov8_parallel3_;
  std::shared_ptr<auto_aim::YOLOV8> yolov8_parallel4_;

  Decider decider_;
  bool stop_flag_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
};

}  // namespace omniperception
#endif