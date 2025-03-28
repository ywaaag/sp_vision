#include "decider.hpp"

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace omniperception
{
Decider::Decider(const std::string & config_path) : detector_(config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  img_width_ = yaml["image_width"].as<double>();
  img_height_ = yaml["image_height"].as<double>();
  fov_h_ = yaml["fov_h"].as<double>();
  fov_v_ = yaml["fov_v"].as<double>();
  new_fov_h_ = yaml["new_fov_h"].as<double>();
  new_fov_v_ = yaml["new_fov_v"].as<double>();
  enemy_color_ =
    (yaml["enemy_color"].as<std::string>() == "red") ? auto_aim::Color::red : auto_aim::Color::blue;
  mode_ = yaml["mode"].as<double>();
}

io::Command Decider::decide(
  auto_aim::YOLOV8 & yolov8, const Eigen::Vector3d & gimbal_pos, io::USBCamera & usbcam1,
  io::USBCamera & usbcam2, io::USBCamera & usbcam3, io::USBCamera & usbcam4)
{
  Eigen::Vector2d delta_angle;
  io::USBCamera * cams[] = {&usbcam1, &usbcam2, &usbcam3, &usbcam4};

  for (int i = 0; i < 4; ++i) {
    cv::Mat usb_img;
    std::chrono::steady_clock::time_point timestamp;
    cams[i]->read(usb_img, timestamp);
    auto armors = yolov8.detect(usb_img);
    auto empty = armor_filter(armors);

    if (!empty) {
      delta_angle = this->delta_angle(armors, cams[i]->device_name);
      tools::logger()->debug(
        "delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}", delta_angle[0],
        delta_angle[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

      return io::Command{
        true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
        tools::limit_rad(delta_angle[1] / 57.3)};
    }
  }

  // 如果没有找到目标，返回默认命令
  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(tools::ThreadSafeQueue<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) {
    return io::Command{false, false, 0, 0};
  }

  DetectionResult dr;
  detection_queue.pop(dr);
  if (dr.armors.empty()) return io::Command{false, false, 0, 0};
  tools::logger()->info(
    "omniperceptron find {},delta yaw is {:.4f}", auto_aim::ARMOR_NAMES[dr.armors.front().name],
    dr.delta_yaw * 57.3);

  return io::Command{true, false, dr.delta_yaw, dr.delta_pitch};
};

Eigen::Vector2d Decider::delta_angle(
  const std::list<auto_aim::Armor> & armors, const std::string & camera)
{
  Eigen::Vector2d delta_angle;
  if (camera == "front_left") {
    delta_angle[0] = 35 + (fov_h_ / 2) - armors.front().center_norm.x * fov_h_;
    delta_angle[1] = -(armors.front().center_norm.y * fov_v_ - fov_v_ / 2);
    return delta_angle;
  } else if (camera == "front_right") {
    delta_angle[0] = -35 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = -(armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2);
    return delta_angle;
  } else if (camera == "back_left") {
    delta_angle[0] = 105 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = -(armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2);
    return delta_angle;
  } else {
    delta_angle[0] = -105 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    delta_angle[1] = -(armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2);
    return delta_angle;
  }
}

bool Decider::armor_filter(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return true;
  // 过滤非敌方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

  // 25赛季没有5号装甲板
  // armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::five; });

  // RMUL只保留1、3、sentry装甲板
  armors.remove_if([&](const auto_aim::Armor & a) {
    return a.name != auto_aim::ArmorName::one && a.name != auto_aim::ArmorName::three &&
           a.name != auto_aim::ArmorName::sentry;
  });

  // 过滤掉刚复活无敌的装甲板
  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(invincible_armor_.begin(), invincible_armor_.end(), a.name) !=
           invincible_armor_.end();
  });

  return armors.empty();
}

void Decider::set_priority(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return;

  const PriorityMap & priority_map = (mode_ == MODE_ONE) ? mode1 : mode2;

  if (!armors.empty()) {
    for (auto & armor : armors) {
      armor.priority = priority_map.at(armor.name);
    }
  }
}

void Decider::sort(tools::ThreadSafeQueue<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) return;

  std::vector<DetectionResult> results;

  // 从队列中取出所有 DetectionResult
  while (!detection_queue.empty()) {
    DetectionResult dr;
    detection_queue.pop(dr);
    results.push_back(dr);
  }

  // 对每个 DetectionResult 调用 armor_filter 和 set_priority
  for (auto & dr : results) {
    armor_filter(dr.armors);
    set_priority(dr.armors);

    // 对每个 DetectionResult 中的 armors 进行排序
    dr.armors.sort(
      [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });
  }

  // 根据优先级对 DetectionResult 进行排序
  std::sort(
    results.begin(), results.end(), [](const DetectionResult & a, const DetectionResult & b) {
      return a.armors.front().priority < b.armors.front().priority;
    });

  // 将排序后的结果重新推入队列
  for (const auto & dr : results) {
    detection_queue.push(dr);
  }
}

Eigen::Vector4d Decider::get_target_info(
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets)
{
  if (armors.empty() || targets.empty()) return Eigen::Vector4d::Zero();

  auto target = targets.front();

  for (const auto & armor : armors) {
    if (armor.name == target.name) {
      return Eigen::Vector4d{
        armor.xyz_in_gimbal[0], armor.xyz_in_gimbal[1], 1,
        static_cast<double>(armor.name) + 1};  //避免歧义+1
    }
  }

  return Eigen::Vector4d::Zero();
}

void Decider::get_invincible_armor(const std::vector<int8_t> & invincible_enemy_ids)
{
  invincible_armor_.clear();

  if (invincible_enemy_ids.empty()) return;

  for (const auto & id : invincible_enemy_ids) {
    tools::logger()->info("invincible armor id: {}", id);
    invincible_armor_.push_back(auto_aim::ArmorName(id - 1));
  }
}

}  // namespace omniperception