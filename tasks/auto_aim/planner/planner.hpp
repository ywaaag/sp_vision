#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>

#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HORIZON = 100;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_dot, pitch, pitch_dot

struct Plan
{
  bool control;
  bool fire;
  float yaw;
  float vyaw;
  float yaw_torque;
  float pitch;
  float vpitch;
  float pitch_torque;
};

class Planner
{
public:
  Planner(const std::string & config_path);

  Plan plan(Target target, io::GimbalState gs);

private:
  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  Eigen::Matrix<double, 2, 1> observe(Target target, double bullet_speed) const;
  Trajectory get_trajectory(Target target, double gimbal_yaw, double bullet_speed);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP