#ifndef AUTO_AIM__AUTO_SHOOT_AIMER_HPP
#define AUTO_AIM__AUTO_SHOOT_AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    const std::list<Target> & targets, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point timestamp, double bullet_speed, bool to_now = true);
  void clear_last();

private:
  double yaw_offset_;
  double pitch_offset_;
  double shooting_angle_;  // aim at -shooting_angle ~ shooting_angle
  double delay_gimbal_;
  double delay_shoot_;
  int lock_id_ = -1;
  /// TODO: aim at next coming armor
  ArmorName last_target_name_ = not_armor;

  AimPoint choose_aim_point(const Target & target);
  AimPoint choose_coming_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP