#include "detector.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{

Detector::Detector(const std::string & config_path, bool debug)
: classifier_(config_path), debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);

  threshold_ = yaml["threshold"].as<double>();
  max_angle_error_ = yaml["max_angle_error"].as<double>() / 57.3;  // degree to rad
  min_lightbar_ratio_ = yaml["min_lightbar_ratio"].as<double>();
  max_lightbar_ratio_ = yaml["max_lightbar_ratio"].as<double>();
  min_lightbar_length_ = yaml["min_lightbar_length"].as<double>();
  min_armor_ratio_ = yaml["min_armor_ratio"].as<double>();
  max_armor_ratio_ = yaml["max_armor_ratio"].as<double>();
  max_side_ratio_ = yaml["max_side_ratio"].as<double>();
  min_confidence_ = yaml["min_confidence"].as<double>();
  max_rectangular_error_ = yaml["max_rectangular_error"].as<double>() / 57.3;  // degree to rad

  save_path_ = "patterns";
  // debug_path_ = "wrong_patterns";
  std::filesystem::create_directory(save_path_);
  // std::filesystem::create_directory(debug_path_);
}

std::list<Armor> Detector::detect(const cv::Mat & bgr_img, int frame_count)
{
  if (bgr_img.empty()) {
    tools::logger()->warn("Empty img!, Usb camera drop!");
    return std::list<Armor>();
  }
  // 彩色图转灰度图
  cv::Mat gray_img;
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);

  // 进行二值化
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);

  // 获取轮廓点
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  // 获取灯条
  std::size_t lightbar_id = 0;
  std::list<Lightbar> lightbars;
  for (const auto & contour : contours) {
    auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = Lightbar(rotated_rect, lightbar_id);

    if (!check_geometry(lightbar)) continue;

    lightbar.color = get_color(bgr_img, contour);
    lightbars.emplace_back(lightbar);
    lightbar_id += 1;
  }

  // 将灯条从左到右排序
  lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

  // 获取装甲板
  std::list<Armor> armors;
  for (auto left = lightbars.begin(); left != lightbars.end(); left++) {
    for (auto right = std::next(left); right != lightbars.end(); right++) {
      if (left->color != right->color) continue;

      auto armor = Armor(*left, *right);
      if (!check_geometry(armor)) continue;

      armor.pattern = get_pattern(bgr_img, armor);
      classifier_.classify(armor);
      if (!check_name(armor)) continue;

      armor.type = get_type(armor);
      if (!check_type(armor)) continue;
      // savewrong(armor);

      armor.center_norm = get_center_norm(bgr_img, armor.center);
      armors.emplace_back(armor);
    }
  }

  // 检查装甲板是否存在共用灯条的情况
  for (auto armor1 = armors.begin(); armor1 != armors.end(); armor1++) {
    for (auto armor2 = std::next(armor1); armor2 != armors.end(); armor2++) {
      if (
        armor1->left.id != armor2->left.id && armor1->left.id != armor2->right.id &&
        armor1->right.id != armor2->left.id && armor1->right.id != armor2->right.id) {
        continue;
      }

      // 装甲板重叠, 保留roi小的
      if (armor1->left.id == armor2->left.id || armor1->right.id == armor2->right.id) {
        auto area1 = armor1->pattern.cols * armor1->pattern.rows;
        auto area2 = armor2->pattern.cols * armor2->pattern.rows;
        if (area1 < area2)
          armor2->duplicated = true;
        else
          armor1->duplicated = true;
      }

      // 装甲板相连，保留置信度大的
      if (armor1->left.id == armor2->right.id || armor1->right.id == armor2->left.id) {
        if (armor1->confidence < armor2->confidence)
          armor1->duplicated = true;
        else
          armor2->duplicated = true;
      }
    }
  }

  armors.remove_if([&](const Armor & a) { return a.duplicated; });

  if (debug_) show_result(binary_img, bgr_img, lightbars, armors, frame_count);

  return armors;
}

bool Detector::check_geometry(const Lightbar & lightbar) const
{
  auto angle_ok = lightbar.angle_error < max_angle_error_;
  auto ratio_ok = lightbar.ratio > min_lightbar_ratio_ && lightbar.ratio < max_lightbar_ratio_;
  auto length_ok = lightbar.length > min_lightbar_length_;
  return angle_ok && ratio_ok && length_ok;
}

bool Detector::check_geometry(const Armor & armor) const
{
  auto ratio_ok = armor.ratio > min_armor_ratio_ && armor.ratio < max_armor_ratio_;
  auto side_ratio_ok = armor.side_ratio < max_side_ratio_;
  auto rectangular_error_ok = armor.rectangular_error < max_rectangular_error_;
  return ratio_ok && side_ratio_ok && rectangular_error_ok;
}

bool Detector::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // 保存不确定的图案，用于分类器的迭代
  if (name_ok && !confidence_ok) save(armor);

  return name_ok && confidence_ok;
}

bool Detector::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
                      armor.name != ArmorName::outpost);

  // 保存异常的图案，用于分类器的迭代
  if (!name_ok) save(armor);

  return name_ok;
}

Color Detector::get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const
{
  int red_sum = 0, blue_sum = 0;

  for (const auto & point : contour) {
    red_sum += bgr_img.at<cv::Vec3b>(point)[2];
    blue_sum += bgr_img.at<cv::Vec3b>(point)[0];
  }

  return blue_sum > red_sum ? Color::blue : Color::red;
}

cv::Mat Detector::get_pattern(const cv::Mat & bgr_img, const Armor & armor) const
{
  // 延长灯条获得装甲板角点
  // 1.125 = 0.5 * armor_height / lightbar_length = 0.5 * 126mm / 56mm
  auto tl = armor.left.center - armor.left.top2bottom * 1.125;
  auto bl = armor.left.center + armor.left.top2bottom * 1.125;
  auto tr = armor.right.center - armor.right.top2bottom * 1.125;
  auto br = armor.right.center + armor.right.top2bottom * 1.125;

  auto roi_left = std::max<int>(std::min(tl.x, bl.x), 0);
  auto roi_top = std::max<int>(std::min(tl.y, tr.y), 0);
  auto roi_right = std::min<int>(std::max(tr.x, br.x), bgr_img.cols);
  auto roi_bottom = std::min<int>(std::max(bl.y, br.y), bgr_img.rows);
  auto roi_tl = cv::Point(roi_left, roi_top);
  auto roi_br = cv::Point(roi_right, roi_bottom);
  auto roi = cv::Rect(roi_tl, roi_br);

  return bgr_img(roi);
}

ArmorType Detector::get_type(const Armor & armor)
{
  auto big = voter_.count(armor.color, armor.name, ArmorType::big);
  auto small = voter_.count(armor.color, armor.name, ArmorType::small);

  /// 优先根据当前armor.ratio判断

  if (armor.ratio > 3.0) {
    // 最多领先100票
    if (big - small < 100) voter_.vote(armor.color, armor.name, ArmorType::big);
    return ArmorType::big;
  }

  if (armor.ratio < 2.5) {
    // 最多领先100票
    if (small - big < 100) voter_.vote(armor.color, armor.name, ArmorType::small);
    return ArmorType::small;
  }

  // 如果无法通过armor.ratio判断，则根据选票决定
  if (big != small) {
    return (big > small) ? ArmorType::big : ArmorType::small;
  }

  /// 如果选票相等，则根据装甲板类型判断

  tools::logger()->debug("[Detector] get armor type by name: {}", ARMOR_NAMES[armor.name]);

  // 英雄、基地只能是大装甲板
  if (armor.name == ArmorName::one || armor.name == ArmorName::base) {
    return ArmorType::big;
  }

  // 工程、哨兵、前哨站只能是小装甲板
  if (
    armor.name == ArmorName::two || armor.name == ArmorName::sentry ||
    armor.name == ArmorName::outpost) {
    return ArmorType::small;
  }

  // 步兵假设为小装甲板
  return ArmorType::small;
}

cv::Point2f Detector::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void Detector::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, armor.pattern);
}

void Detector::savewrong(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", debug_path_, armor.name, file_name);
  cv::imwrite(img_path, armor.pattern);
}

void Detector::show_result(
  const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars,
  const std::list<Armor> & armors, int frame_count) const
{
  auto detection = bgr_img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

  for (const auto & lightbar : lightbars) {
    auto info = fmt::format(
      "{:.1f} {:.1f} {:.1f} {}", lightbar.angle_error * 57.3, lightbar.ratio, lightbar.length,
      COLORS[lightbar.color]);
    tools::draw_text(detection, info, lightbar.top, {0, 255, 255});
    tools::draw_points(detection, lightbar.points, {0, 255, 255}, 3);
  }

  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {:.2f} {:.1f} {:.2f} {} {}", armor.ratio, armor.side_ratio,
      armor.rectangular_error * 57.3, armor.confidence, ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.left.bottom, {0, 255, 0});
  }

  cv::Mat binary_img2;
  cv::resize(binary_img, binary_img2, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
  cv::resize(detection, detection, {}, 0.5, 0.5);     // 显示时缩小图片尺寸

  cv::imshow("threshold", binary_img2);
  cv::imshow("detection", detection);
}

}  // namespace auto_aim