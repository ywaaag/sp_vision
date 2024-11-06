#include "decider.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace omniperception
{
Decider::Decider(const std::string & config_path)
: state_("no_target"),
  shift_count_(0),
  min_shift_count_(2),
  detector_(config_path),
  track_state_(0),
  temp_lost_count_(0)
{
  auto yaml = YAML::LoadFile(config_path);
  img_width_ = yaml["image_width"].as<double>();
  img_height_ = yaml["image_height"].as<double>();
  fov_h_ = yaml["fov_h"].as<double>();
  fov_v_ = yaml["fov_v"].as<double>();
  enemy_color_ =
    (yaml["enemy_color"].as<std::string>() == "red") ? auto_aim::Color::red : auto_aim::Color::blue;
  min_find_count_ = yaml["min_find_count"].as<int>();
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
}

Eigen::Vector2d Decider::delta_angle(
  const std::list<auto_aim::Armor> & armors, const std::string & camera)
{
  Eigen::Vector2d delta_angle;
  if (camera == "left") {
    delta_angle[0] = 90 + (fov_h_ / 2) - armors.front().center_norm.x * fov_h_;
    delta_angle[1] = -(armors.front().center_norm.y * fov_v_ - fov_v_ / 2);
    return delta_angle;
  } else if (camera == "right") {
    delta_angle[0] = -90 + (fov_h_ / 2) - armors.front().center_norm.x * fov_h_;
    delta_angle[1] = -(armors.front().center_norm.y * fov_v_ - fov_v_ / 2);
    return delta_angle;
  } else {
    delta_angle[0] = -180 + (fov_h_ / 2) - armors.front().center_norm.x * fov_h_;
    delta_angle[1] = -(armors.front().center_norm.y * fov_v_ - fov_v_ / 2);
    return delta_angle;
  }
}

bool Decider::armor_filter(std::list<auto_aim::Armor> & armors, const std::string & armor_omit)
{
  // 过滤友方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // RMUC过滤前哨站、基地、哨兵
  armors.remove_if([&](const auto_aim::Armor & a) {
    return a.name == auto_aim::ArmorName::outpost || a.name == auto_aim::ArmorName::base ||
           a.name == auto_aim::ArmorName::sentry;
  });

  // 过滤掉刚复活无敌的装甲板
  if (!armor_omit.empty() || armor_omit != "0,") {
    std::vector<std::string> non_zero_numbers;
    std::vector<std::string> numbers;
    std::stringstream ss(armor_omit);
    std::string token;
    while (std::getline(ss, token, ',')) {
      numbers.push_back(token);
    }
    for (const std::string & num : numbers) {
      if (num != "0") {
        non_zero_numbers.push_back(num);
      }
    }
    armors.remove_if([&](const auto_aim::Armor & a) {
      std::string armor_name = std::to_string(static_cast<int>(a.name) + 1);
      return std::find(non_zero_numbers.begin(), non_zero_numbers.end(), armor_name) !=
             non_zero_numbers.end();
    });
  }

  return armors.empty();
}

void Decider::set_priority(std::list<auto_aim::Armor> & armors, const int mode)
{
  const PriorityMap & priority_map = (mode == MODE_ONE) ? mode1 : mode2;

  if (!armors.empty()) {
    for (auto & armor : armors) {
      armor.priority = priority_map.at(armor.name);
    }
  }
}

std::string Decider::state() const { return state_; }

std::string Decider::track_target() const { return track_target_; }

bool Decider::decide(
  const cv::Mat & img, const std::string & device_name, const int & frame_count,
  const Eigen::Vector3d & gimbal_pos, io::Command & command,
  const auto_aim::ArmorPriority & priority, bool use_prority)
{
  auto armors = detector_.detect(img, frame_count);
  auto empty = armor_filter(armors);
  if (!empty) {
    // 使用优先级模式
    if (use_prority) {
      set_priority(armors, 1);
      // 优先选择靠近图像中心的装甲板
      armors.sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) {
        cv::Point2f img_center(1920 / 2, 1080 / 2);
        auto distance_1 = cv::norm(a.center - img_center);
        auto distance_2 = cv::norm(b.center - img_center);
        return distance_1 < distance_2;
      });
      // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
      armors.sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) {
        return a.priority < b.priority;
      });
      // 判断是否切换
      if (armors.front().priority < priority) {
        Eigen::Vector2d angle = delta_angle(armors, device_name);
        tools::logger()->debug(
          "shift to high priority target! delta yaw:{:.2f},target pitch:{:.2f},armor "
          "number:{},armor name:{}",
          angle[0], angle[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);
        track_target_ = auto_aim::ARMOR_NAMES[armors.front().name];
        command = {
          true, false, tools::limit_rad(gimbal_pos[0] + angle[0] / 57.3),
          tools::limit_rad(angle[1] / 57.3)};
        return true;
      } else {
        return false;
      }
    }
    // 使用常规模式
    else {
      Eigen::Vector2d angle = delta_angle(armors, device_name);
      tools::logger()->debug(
        "delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}", angle[0], angle[1],
        armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);
      command = {
        true, false, tools::limit_rad(gimbal_pos[0] + angle[0] / 57.3),
        tools::limit_rad(angle[1] / 57.3)};
      return true;
    }
  } else {
    return false;
  }
}

bool Decider::check_perception(
  const std::string & str1, const std::string & str2, const std::string & str3)
{
  return (str1 == str2 || str2 == str3 || str1 == str3);
}

}  // namespace omniperception