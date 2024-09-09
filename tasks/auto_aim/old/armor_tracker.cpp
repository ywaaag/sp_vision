#include "armor_tracker.hpp"

#include "tools/math_tools.hpp"

namespace auto_aim
{
ArmorTracker::ArmorTracker(Solver & solver) : solver_{solver}, state_{"lost"} {}

std::string ArmorTracker::state() const { return state_; }

ArmorTarget ArmorTracker::target() const
{
  auto x = ekf_.x[0];
  auto y = ekf_.x[2];
  auto z = ekf_.x[4];
  auto vx = ekf_.x[1];
  auto vy = ekf_.x[3];
  auto vz = ekf_.x[5];
  return {{x, y, z}, {vx, vy, vz}};
}

void ArmorTracker::init(
  std::vector<Armor> & armors, std::chrono::steady_clock::time_point timestamp)
{
  if (armors.empty()) {
    update_state(false);
    return;
  }

  auto & armor = armors[0];
  target_name_ = armor.name;
  last_timestamp_ = timestamp;

  solver_.solve(armor);
  const auto & xyz = armor.xyz_in_world;
  Eigen::VectorXd x0{{xyz[0], 0, xyz[1], 0, xyz[2], 0}};
  Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  ekf_ = tools::ExtendedKalmanFilter(x0, P0);
  update_state(true);
}

void ArmorTracker::track(
  std::vector<Armor> & armors, std::chrono::steady_clock::time_point timestamp)
{
  auto dt = tools::delta_time(timestamp, last_timestamp_);
  last_timestamp_ = timestamp;

  // clang-format off
  Eigen::MatrixXd F{
    {1, dt, 0,  0, 0,  0},
    {0,  1, 0,  0, 0,  0},
    {0,  0, 1, dt, 0,  0},
    {0,  0, 0,  1, 0,  0},
    {0,  0, 0,  0, 1, dt},
    {0,  0, 0,  0, 0,  1},
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  // clang-format off
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  Eigen::MatrixXd Q{
    {a, b, 0, 0, 0, 0},
    {b, c, 0, 0, 0, 0},
    {0, 0, a, b, 0, 0},
    {0, 0, b, c, 0, 0},
    {0, 0, 0, 0, a, b},
    {0, 0, 0, 0, b, c}
  };
  Q *= 100; // 加速度方差
  // clang-format on

  ekf_.predict(F, Q);

  auto found = std::find_if(
    armors.begin(), armors.end(), [&](const Armor & a) { return a.name == target_name_; });
  if (found == armors.end()) {
    update_state(false);
    return;
  }

  auto & armor = *found;
  solver_.solve(armor);

  // clang-format off
  Eigen::MatrixXd H_xyz_in_world{
    {1, 0, 0, 0, 0, 0},
    {0, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 1, 0}
  };
  // clang-format on

  Eigen::MatrixXd H = tools::xyz2ypd_jacobian(H_xyz_in_world * ekf_.x) * H_xyz_in_world;
  Eigen::VectorXd R_dig{{4e-3, 4e-3, 1}};
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [H_xyz_in_world](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd xyz_in_world = H_xyz_in_world * x;
    return tools::xyz2ypd(xyz_in_world);
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    return c;
  };

  ekf_.update(armor.ypd_in_world, H, R, h, z_subtract);
  update_state(true);
}

void ArmorTracker::update_state(bool matched)
{
  // 有限状态机

  if (state_ == "lost") {
    if (!matched) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (matched) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (matched) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "temp_lost") {
    if (matched) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

}  // namespace auto_aim