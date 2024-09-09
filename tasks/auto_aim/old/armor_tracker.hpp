#ifndef AUTO_AIM__ARMOR_TRACKER_HPP
#define AUTO_AIM__ARMOR_TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <vector>

#include "armor.hpp"
#include "armor_target.hpp"
#include "solver.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
class ArmorTracker
{
public:
  explicit ArmorTracker(Solver & solver);

  std::string state() const;

  ArmorTarget target() const;

  void init(std::vector<Armor> & armors, std::chrono::steady_clock::time_point timestamp);

  void track(std::vector<Armor> & armors, std::chrono::steady_clock::time_point timestamp);

private:
  Solver & solver_;
  int min_detect_count_ = 5;
  int max_temp_lost_count_ = 10;

  std::string state_;
  int temp_lost_count_;
  int detect_count_;

  std::string target_name_;
  std::chrono::steady_clock::time_point last_timestamp_;
  tools::ExtendedKalmanFilter ekf_;

  void update_state(bool matched);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_TRACKER_HPP