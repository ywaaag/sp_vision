#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include <functional>

namespace std {
    template <>
    struct hash<std::pair<auto_aim::ArmorName, auto_aim::ArmorType>> {
        size_t operator()(const std::pair<auto_aim::ArmorName, auto_aim::ArmorType>& key) const {
            size_t hash_value = 0;
            hash_value ^= std::hash<auto_aim::ArmorName>{}(key.first) + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
            hash_value ^= std::hash<auto_aim::ArmorType>{}(key.second) + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
            return hash_value;
        }
    };
}

namespace auto_aim
{
  using TargetKey = std::pair<ArmorName, ArmorType>;
    
class Tracker
{
public:
  // 此处target用于初始化target_ 无实际意义 随便传入一个target即可
  Tracker(const std::string & config_path, Solver & solver, Target & target);

  std::string state() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

private:
  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_;
  std::unordered_map<TargetKey, Target> targets_;
//   std::list<Target> targets_;
  Target & target_;
  std::chrono::steady_clock::time_point last_timestamp_;

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP