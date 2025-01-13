#ifndef OMNIPERCEPTION__DETECTION_HPP
#define OMNIPERCEPTION__DETECTION_HPP

#include <chrono>
#include <list>

#include "tasks/auto_aim_sentry/armor.hpp"

namespace omniperception
{
struct DetectionResult
{
  std::list<auto_aim::Armor> armors;
  std::chrono::steady_clock::time_point timestamp;
  double delta_yaw;
  double delta_pitch;
};
}  // namespace omniperception

#endif