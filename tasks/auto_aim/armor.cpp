#include "armor.hpp"

#include <algorithm>
#include <cmath>

namespace auto_aim
{
Lightbar::Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id) : id(id)
{
  std::vector<cv::Point2f> corners(4);
  rotated_rect.points(&corners[0]);
  std::sort(corners.begin(), corners.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  center = rotated_rect.center;
  top = (corners[0] + corners[1]) / 2;
  bottom = (corners[2] + corners[3]) / 2;
  top2bottom = bottom - top;

  points.emplace_back(top);
  points.emplace_back(bottom);

  auto width = cv::norm(corners[0] - corners[1]);
  angle = std::atan2(top2bottom.y, top2bottom.x);
  angle_error = std::abs(angle - CV_PI / 2);
  length = cv::norm(top2bottom);
  ratio = length / width;
}

Armor::Armor(const Lightbar & left, const Lightbar & right)
: left(left), right(right), duplicated(false)
{
  color = left.color;
  center = (left.center + right.center) / 2;

  points.emplace_back(left.top);
  points.emplace_back(right.top);
  points.emplace_back(right.bottom);
  points.emplace_back(left.bottom);

  auto left2right = right.center - left.center;
  auto width = cv::norm(left2right);
  auto max_lightbar_length = std::max(left.length, right.length);
  auto min_lightbar_length = std::min(left.length, right.length);
  ratio = width / max_lightbar_length;
  side_ratio = max_lightbar_length / min_lightbar_length;

  auto roll = std::atan2(left2right.y, left2right.x);
  auto left_rectangular_error = std::abs(left.angle - roll - CV_PI / 2);
  auto right_rectangular_error = std::abs(right.angle - roll - CV_PI / 2);
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);
}

}  // namespace auto_aim