#ifndef AUTO_AIM__ARMOR_AIMER_HPP
#define AUTO_AIM__ARMOR_AIMER_HPP

#include <chrono>

#include "armor_target.hpp"
#include "io/command.hpp"

namespace auto_aim
{

class ArmorAimer
{
public:
  explicit ArmorAimer(const std::string & config_path);
  io::Command aim(
    ArmorTarget target, std::chrono::steady_clock::time_point timestamp, double bullet_speed);

private:
  double yaw_offset_;
  double pitch_offset_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_AIMER_HPP