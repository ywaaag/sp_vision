#include "usbcamera.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
USBCamera::USBCamera(const std::string & open_name, const std::string & config_path)
: open_name_(open_name), quit_(false), ok_(false), queue_(1), open_count_(0)
{
  auto yaml = YAML::LoadFile(config_path);
  image_width_ = yaml["image_width"].as<double>();
  image_height_ = yaml["image_height"].as<double>();
  new_image_height_ = yaml["new_image_height"].as<double>();
  new_image_width_ = yaml["new_image_width"].as<double>();
  usb_exposure_ = yaml["usb_exposure"].as<double>();
  new_usb_exposure_ = yaml["new_usb_exposure"].as<double>();
  usb_frame_rate_ = yaml["usb_frame_rate"].as<double>();
  usb_gamma_ = yaml["usb_gamma"].as<double>();
  usb_gain_ = yaml["usb_gain"].as<double>();
  try_open();

  // 守护线程
  daemon_thread_ = std::thread{[this] {
    // tools::logger()->info("daemon thread start");
    while (!quit_) {
      std::this_thread::sleep_for(100ms);

      if (ok_) continue;

      if (open_count_ > 20) {
        tools::logger()->warn("Give up to open {} USB camera", this->device_name);
        quit_ = true;

        {
          std::lock_guard<std::mutex> lock(cap_mutex_);
          close();  // 先关闭摄像头
        }

        if (capture_thread_.joinable()) {
          tools::logger()->warn("Stopping capture thread");
          capture_thread_.join();
        }

        break;
      }

      if (capture_thread_.joinable()) capture_thread_.join();

      {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        close();
      }
      try_open();
    }
    // tools::logger()->info("daemon thread exit");
  }};
}

USBCamera::~USBCamera()
{
  quit_ = true;
  {
    std::lock_guard<std::mutex> lock(cap_mutex_);
    close();
  }
  if (daemon_thread_.joinable()) daemon_thread_.join();
  if (capture_thread_.joinable()) capture_thread_.join();
  tools::logger()->info("USBCamera destructed.");
}

cv::Mat USBCamera::read()
{
  std::lock_guard<std::mutex> lock(cap_mutex_);
  if (!cap_.isOpened()) {
    tools::logger()->warn("Failed to read {} USB camera", this->device_name);
    return cv::Mat();
  }
  cap_ >> img_;
  return img_;
}

void USBCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

void USBCamera::open()
{
  std::lock_guard<std::mutex> lock(cap_mutex_);
  std::string true_device_name = "/dev/" + open_name_;
  cap_.open(true_device_name, cv::CAP_V4L);
  if (!cap_.isOpened()) {
    tools::logger()->warn("Failed to open USB camera");
    return;
  }
  sharpness_ = cap_.get(cv::CAP_PROP_SHARPNESS);
  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap_.set(cv::CAP_PROP_FPS, usb_frame_rate_);
  cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
  cap_.set(cv::CAP_PROP_GAMMA, usb_gamma_);
  cap_.set(cv::CAP_PROP_GAIN, usb_gain_);

  if (sharpness_ == 2) {
    device_name = "front_left";
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    cap_.set(cv::CAP_PROP_EXPOSURE, usb_exposure_);
  } else if (sharpness_ == 3) {
    device_name = "front_right";
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    cap_.set(cv::CAP_PROP_EXPOSURE, usb_exposure_);
  } else if (sharpness_ == 4) {
    device_name = "back_left";
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, new_image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, new_image_height_);
    cap_.set(cv::CAP_PROP_EXPOSURE, new_usb_exposure_);

  } else {
    device_name = "back_right";
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, new_image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, new_image_height_);
    cap_.set(cv::CAP_PROP_EXPOSURE, new_usb_exposure_);
  }

  tools::logger()->info("{} USBCamera opened", device_name);
  // tools::logger()->info("USBCamera exposure time:{}", cap_.get(cv::CAP_PROP_EXPOSURE));
  // tools::logger()->info("USBCamera fps:{}", cap_.get(cv::CAP_PROP_FPS));
  // tools::logger()->info("USBCamera gamma:{}", cap_.get(cv::CAP_PROP_GAMMA));

  // 取图线程
  capture_thread_ = std::thread{[this] {
    ok_ = true;
    // tools::logger()->info("capture thread start");
    std::this_thread::sleep_for(50ms);
    while (!quit_) {
      std::this_thread::sleep_for(1ms);

      cv::Mat img;
      bool success;
      {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        if (!cap_.isOpened()) {
          break;
        }
        success = cap_.read(img);
      }

      if (!success) {
        tools::logger()->warn("Failed to read frame, exiting capture thread");
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();
      queue_.push({img, timestamp});
    }
    ok_ = false;
    // tools::logger()->info("capture thread exit");
  }};
}

void USBCamera::try_open()
{
  try {
    open();
    open_count_++;
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void USBCamera::close()
{
  if (cap_.isOpened()) {
    cap_.release();
    tools::logger()->info("USB camera released.");
  }
}

}  // namespace io