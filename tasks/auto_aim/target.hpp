#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
enum State
{
  detecting,
  tracking,
  lost,
  temp_lost
};
const std::vector<std::string> state_names_ = {"detecting", "tracking", "lost", "temp_lost"};

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  bool jumped;
  int last_id;  // debug only
  State state;

  Target() = default;
  Target(ArmorName armor_name);

  void update(
    std::list<Armor> & armors_of_this_target, std::chrono::steady_clock::time_point t_img);

  void predict(std::chrono::steady_clock::time_point t);
  Eigen::VectorXd ekf_x() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

private:
  int armor_num_;
  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_last_seen_;
  std::chrono::steady_clock::time_point t_ekf_;

  int consecutive_detect_frame_cnt_;
  void reset_ekf(const Armor & armor, std::chrono::steady_clock::time_point t_img);

  Eigen::MatrixXd P0_;
  double r0_;
  double v1_;  // 加速度方差
  double v2_;  // 角加速度方差

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP