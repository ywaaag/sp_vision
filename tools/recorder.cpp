#include "recorder.hpp"

#include <fmt/chrono.h>

#include <filesystem>
#include <string>

#include "math_tools.hpp"

namespace tools
{
Recorder::Recorder(double fps) : init_(false), fps_(fps)
{
  start_time_ = std::chrono::steady_clock::now();
  last_time_ = start_time_;

  auto folder_path = "records";
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  text_path_ = fmt::format("{}/{}.txt", folder_path, file_name);
  video_path_ = fmt::format("{}/{}.avi", folder_path, file_name);

  std::filesystem::create_directory(folder_path);
}

Recorder::~Recorder()
{
  if (!init_) return;
  text_writer_.close();
  video_writer_.release();
}

void Recorder::record(
  const cv::Mat & img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp)
{
  if (!init_) init(img);

  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  video_writer_.write(img);

  // 输出顺序为wxyz
  Eigen::Vector4d xyzw = q.coeffs();
  auto since_begin = tools::delta_time(timestamp, start_time_);
  text_writer_ << fmt::format("{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
}

void Recorder::init(const cv::Mat & img)
{
  text_writer_.open(text_path_);

  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  video_writer_ = cv::VideoWriter(video_path_, fourcc, fps_, img.size());

  init_ = true;
}

}  // namespace tools
