#ifndef AUTO_BUFF__AIMER_HPP
#define AUTO_BUFF__AIMER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <vector>

#include "buff_target.hpp"
#include "buff_type.hpp"
#include "io/command.hpp"

namespace auto_buff
{
class Aimer
{
public:
  Aimer(const std::string & config_path);

  io::Command aim(
    Target & target, std::chrono::steady_clock::time_point & timestamp, double bullet_speed,
    bool to_now = true);

  double angle;      ///
  double t_gap = 0;  ///

private:
  enum STATUS
  {
    SEND_ANGLE,
    SEND_FIRE,
    WAIT,
  };
  SmallTarget target_;
  double yaw_offset_;
  double pitch_offset_;
  STATUS status_ = SEND_ANGLE;
  const double AIM_TIME = 0.150;
  const double WAIT_TIME = 0.100;
  const double PREDICT_TIME = 0.300;

  bool get_send_angle(
    Target & target, const double & detect_now_gap, const double bullet_speed, const bool to_now,
    double & yaw, double & pitch);

  void update_status_();

  void reset_status_();
};
}  // namespace auto_buff
#endif  // AUTO_AIM__AIMER_HPP