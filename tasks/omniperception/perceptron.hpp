#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <chrono>
#include <list>
#include <memory>

#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim_sentry/armor.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/thread_pool.hpp"
#include "tools/thread_safe_queue.hpp"

namespace omniperception
{
struct DetectionResult
{
  std::list<auto_aim::Armor> armors;
  std::chrono::steady_clock::time_point timestamp;
  double delta_yaw;
  double delta_pitch;
};

class Perceptron
{
public:
  Perceptron(
    io::USBCamera & usbcam1, io::USBCamera & usbcam2, io::USBCamera & usbcam3,
    io::USBCamera & usbcam4, const std::string & config_path);

  ~Perceptron();

  const tools::ThreadSafeQueue<DetectionResult> & get_detection_queue() const;

private:
  tools::ThreadPool thread_pool_;
  tools::ThreadSafeQueue<DetectionResult> detection_queue_;

  Decider decider_;
  bool stop_flag_;
  std::mutex mutex_;
  std::condition_variable condition_;
};

}  // namespace omniperception
#endif