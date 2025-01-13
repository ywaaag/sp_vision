#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <chrono>
#include <list>
#include <memory>

#include "tasks/auto_aim_sentry/armor.hpp"

namespace omniperception
{
class Perceptron
{
};

struct DetectionResult
{
  std::list<auto_aim::Armor> armors;
  std::chrono::steady_clock::time_point timestamp;
  double delta_yaw;
  double delta_pitch;
};

}  // namespace omniperception
#endif