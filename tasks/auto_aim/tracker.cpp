#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver, Target & target)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  target_(target),
  last_timestamp_(std::chrono::steady_clock::now())
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;

  targets_[{ArmorName::one, ArmorType::big}] = Target(ArmorName::one, ArmorType::big, 4); // 英雄
  targets_[{ArmorName::two, ArmorType::small}] = Target(ArmorName::two, ArmorType::small, 4); // 工程
  // targets_[{ArmorName::three, ArmorType::big}] = Target(ArmorName::three, ArmorType::big, 2); // 3号平衡
  // targets_[{ArmorName::four, ArmorType::big}] = Target(ArmorName::four, ArmorType::big, 2); // 4号平衡
  // targets_[{ArmorName::five, ArmorType::big}] = Target(ArmorName::five, ArmorType::big, 2); // 5号平衡
  targets_[{ArmorName::three, ArmorType::small}] = Target(ArmorName::three, ArmorType::small, 4); // 3号步兵
  targets_[{ArmorName::four, ArmorType::small}] = Target(ArmorName::four, ArmorType::small, 4); // 4号步兵
  targets_[{ArmorName::five, ArmorType::small}] = Target(ArmorName::five, ArmorType::small, 4); // 5号步兵
  // targets_[{ArmorName::base, ArmorType::small}] = Target(ArmorName::base, ArmorType::small, 3); // 基地小
  targets_[{ArmorName::base, ArmorType::big}] = Target(ArmorName::base, ArmorType::big, 3); // 基地大
  targets_[{ArmorName::outpost, ArmorType::small}] = Target(ArmorName::outpost, ArmorType::small, 3);  // 前哨站
}

std::string Tracker::state() const { return state_; }

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 过滤掉我方颜色的装甲板
  if (use_enemy_color) armors.remove_if([&](const Armor & a) { return a.color != enemy_color_; });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    // cv::Point2f img_center(1280 / 2, 1024 / 2);  // TODO
    auto img_center_norm = cv::Point2f(0.5, 0.5);
    auto distance_1 = cv::norm(a.center_norm - img_center_norm);
    auto distance_2 = cv::norm(b.center_norm - img_center_norm);
    return distance_1 < distance_2;
  });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  } else if (!armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("switch target to {}", ARMOR_NAMES[armors.front().name]);
  } else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  solver_.solve(armor);

  auto armorNameAndType = std::make_pair(armor.name, armor.type);
  if(targets_.find(armorNameAndType) != targets_.end()) target_ = targets_.at(armorNameAndType);
  target_.setEkf(armor, t); // 由于切换了装甲板，需要重新设定EKF的初始参数
  return true;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  int found_count = 0;
  for (const auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    found_count++;
  }

  if (found_count > 2) {
    tools::logger()->warn("More than 2 target's armors!");
    return false;  // TODO 全部抛弃过于保守且无法判断单一误识别情况，应和ekf结合起来判断
  }
  if (found_count == 0) return false;

  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;

    solver_.solve(armor);

    target_.update(armor);
  }

  return true;
}

}  // namespace auto_aim