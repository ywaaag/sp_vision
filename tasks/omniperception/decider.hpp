#ifndef AUTO_AIM__DECIDER_HPP
#define AUTO_AIM__DECIDER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <iostream>
#include <list>
#include <unordered_map>

#include "io/command.hpp"
#include "tasks/auto_aim_sentry/armor.hpp"
#include "tasks/auto_aim_sentry/yolov8.hpp"

namespace omniperception
{
class Decider
{
public:
  explicit Decider(const std::string & config_path);
  enum PriorityMode
  {
    MODE_ONE = 1,
    MODE_TWO
  };
  Eigen::Vector2d delta_angle(
    const std::list<auto_aim::Armor> & armors, const std::string & camera);
  bool armor_filter(std::list<auto_aim::Armor> & armors, const std::string & armor_omit = "0,");
  void set_priority(std::list<auto_aim::Armor> & armors, const int mode);
  bool decide(
    const cv::Mat & img, const std::string & device_name, const int & frame_count,
    const Eigen::Vector3d & gimbal_pos, io::Command & command,
    const auto_aim::ArmorPriority & priority = auto_aim::fifth, bool use_prority = false);
  bool check_perception(
    const std::string & str1, const std::string & str2, const std::string & str3);
  std::string state() const;
  std::string track_target() const;

private:
  int img_width_;
  int img_height_;
  double fov_h_;
  double fov_v_;
  int shift_count_, min_shift_count_, find_count_, min_find_count_, min_detect_count_, track_state_,
    temp_lost_count_, max_temp_lost_count_;
  std::string state_, track_target_;
  auto_aim::Color enemy_color_;
  auto_aim::YOLOV8 detector_;

  // 定义ArmorName到ArmorPriority的映射类型
  using PriorityMap = std::unordered_map<auto_aim::ArmorName, auto_aim::ArmorPriority>;

  const PriorityMap mode1 = {
    {auto_aim::ArmorName::one, auto_aim::ArmorPriority::first},
    {auto_aim::ArmorName::two, auto_aim::ArmorPriority::forth},
    {auto_aim::ArmorName::three, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::four, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::five, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::fifth},
    {auto_aim::ArmorName::base, auto_aim::ArmorPriority::fifth},
    {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::fifth}};

  const PriorityMap mode2 = {
    {auto_aim::ArmorName::two, auto_aim::ArmorPriority::first},
    {auto_aim::ArmorName::one, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::three, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::four, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::five, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::base, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::third}};
};

}  // namespace omniperception

#endif