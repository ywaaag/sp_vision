#ifndef AUTO_BUFF__TRACK_HPP
#define AUTO_BUFF__TRACK_HPP

#include <yaml-cpp/yaml.h>

#include <deque>
#include <optional>

#include "buff_type.hpp"
#include "tools/img_tools.hpp"

const int LOSE_MAX = 20;  // 丢失的阙值
const cv::Scalar DETECTOR_COLOR_DEBUG = cv::Scalar(0, 255, 0);
const cv::Scalar DETECTOR_COLOR_KEY = cv::Scalar(0, 0, 255);
namespace auto_buff
{
class Buff_Detector
{
public:
  Buff_Detector(const std::string & config);

  std::optional<PowerRune> detect(cv::Mat & bgr_img);

private:
  void handle_img(const cv::Mat & bgr_img, cv::Mat & handled_img);

  std::optional<FanBlade> detect_fanblades(const cv::Mat & handled_img);
  bool detect_fanblades_head(const cv::Mat & handled_img, std::vector<cv::Rect> & head_rects);
  bool detect_fanblades_body(
    const cv::Mat & handled_img, cv::Point2f body_box[4], int & fanblades_angle);

  cv::Point2f detect_r_center(FanBlade & fanblade, const cv::Mat & handled_img);

  void handle_lose();

  std::string enemy_color_;
  int contrast_;
  int brightness_;
  int morphology_size_;
  int dilate_size_;
  // int canny_low_threshold_;
  // int canny_high_threshold_;
  int R_contours_min_area_;
  int R_contours_max_area_;
  int fanblades_head_contours_min_area_;
  int fanblades_head_contours_max_area_;
  int fanblades_body_contours_min_area_;
  int fanblades_body_contours_max_area_;
  int brightness_threshold_;
  cv::Mat standard_fanblade;
  cv::Size standard_fanblade_size;

  cv::Point2f head_center, body_center;
  double head_radius;

  cv::Mat output;

  Track_status status_;
  int lose_;  // 丢失的次数
  double lastlen_;
  std::optional<PowerRune> last_powerrune_ = std::nullopt;
};
}  // namespace auto_buff
#endif  // DETECTOR_HPP