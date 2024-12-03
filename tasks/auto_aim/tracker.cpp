#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver}, last_timestamp_(std::chrono::steady_clock::now())
{
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  for (int i = 0; i < 8; i++) {
    targets_.emplace_back(Target(ArmorName(i)));
  }
}

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t_img)
{
  // 过滤掉我方颜色的装甲板
  armors.remove_if([&](const Armor & a) { return a.color != enemy_color_; });

  // 过滤掉前哨站
  // armors.remove_if([](const Armor & a) { return a.name == ArmorName::outpost; });

  std::vector<std::list<Armor>> armor_lists(8);

  /// TODO: 误识别检测-看到三块同类型装甲板
  for (auto armor : armors) {
    solver_.solve(armor);
    armor_lists[armor.name].emplace_back(armor);
  }

  for (int i = 0; i < targets_.size(); i++) {
    targets_[i].update(armor_lists[i], t_img);
  }

  std::list<Target> traking_targets;

  for (auto target : targets_) {
    tools::logger()->info("    {}", state_names_[target.state]);
    if (target.state == tracking) traking_targets.emplace_back(target);
  }

  return traking_targets;
}

}  // namespace auto_aim