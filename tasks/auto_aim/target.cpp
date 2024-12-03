#include "target.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(ArmorName armor_name)
: name(armor_name), state(lost), jumped(false), last_id(0), consecutive_detect_frame_cnt_(0)
{
  /// TODO: 从一个未初始化的target中取ekf_x会导致错误  /// 非常不安全！
  if (armor_name == ArmorName::outpost) {
    armor_num_ = 3;
    armor_type = small;
    P0_ = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}}.asDiagonal();
    r0_ = 0.28;
    v1_ = 10;
    v2_ = 40;
  } else if (armor_name == ArmorName::base) {
    armor_num_ = 3;
    armor_type = big;
    P0_ = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}}.asDiagonal();
    r0_ = 0.3205;
    v1_ = 10;
    v2_ = 40;
  } else if (armor_name == ArmorName::one) {  // hero
    armor_num_ = 4;
    armor_type = big;
    P0_ = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}}.asDiagonal();
    r0_ = 0.3205;
    v1_ = 10;
    v2_ = 40;
  } else {  // standard
    armor_num_ = 4;
    armor_type = small;
    P0_ = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}}.asDiagonal();
    r0_ = 0.3;
    v1_ = 100;
    v2_ = 400;
  }
}

void Target::predict(std::chrono::steady_clock::time_point t_img)
{
  auto dt = tools::delta_time(t_img, t_ekf_);
  t_ekf_ = t_img;
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1_, b * v1_,       0,       0,       0,       0,       0,       0, 0, 0, 0},
    {b * v1_, c * v1_,       0,       0,       0,       0,       0,       0, 0, 0, 0},
    {      0,       0, a * v1_, b * v1_,       0,       0,       0,       0, 0, 0, 0},
    {      0,       0, b * v1_, c * v1_,       0,       0,       0,       0, 0, 0, 0},
    {      0,       0,       0,       0, a * v1_, b * v1_,       0,       0, 0, 0, 0},
    {      0,       0,       0,       0, b * v1_, c * v1_,       0,       0, 0, 0, 0},
    {      0,       0,       0,       0,       0,       0, a * v2_, b * v2_, 0, 0, 0},
    {      0,       0,       0,       0,       0,       0, b * v2_, c * v2_, 0, 0, 0},
    {      0,       0,       0,       0,       0,       0,       0,       0, 0, 0, 0},
    {      0,       0,       0,       0,       0,       0,       0,       0, 0, 0, 0},
    {      0,       0,       0,       0,       0,       0,       0,       0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  ekf_.predict(F, Q, f);
}

void Target::reset_ekf(const Armor & armor, const std::chrono::steady_clock::time_point t_img)
{
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r0_ * std::cos(ypr[0]);
  auto center_y = xyz[1] + r0_ * std::sin(ypr[0]);
  auto center_z = xyz[2];
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 2.5, r0_, 0, 0}};

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };
  ekf_ = tools::ExtendedKalmanFilter(x0, P0_, x_add);
  t_ekf_ = t_img;
}

void Target::update(std::list<Armor> & armors, std::chrono::steady_clock::time_point t_img)
{
  /// lost - detecting - tracking
  /// 到lost的转换是依据时间完成的

  if (armors.size() > 2) {  // 保留原有状态
    /// TODO: 结合EKF判断
    tools::logger()->debug("  -> see more than 2 {} s", ARMOR_NAMES[name]);
    return;
  }

  auto dt_since_last_seen = tools::delta_time(t_img, t_last_seen_);
  if (!armors.empty()) t_last_seen_ = t_img;

  bool time_out = name == outpost ? (dt_since_last_seen > 0.6)  // 三板的前哨站更容易长期看不见
                                  : (dt_since_last_seen > 0.2);

  /// TODO: ugly code
  if (state != lost && (diverged() || time_out)) {
    tools::logger()->debug("  -> {} lost: {:.3f}s", ARMOR_NAMES[name], dt_since_last_seen);
    state = lost;
    consecutive_detect_frame_cnt_ = 0;
  }
  if (state == lost) {
    if (armors.empty()) return;
    tools::logger()->debug("  -> {} reset ", ARMOR_NAMES[name]);  //lost -> detecting
    auto armor = armors.front();
    armors.pop_front();
    state = detecting;
    consecutive_detect_frame_cnt_ = 1;
    reset_ekf(armor, t_img);  ///取出首个装甲板重置滤波器
  }
  if (
    state == detecting && !armors.empty() &&
    ++consecutive_detect_frame_cnt_ > 5)  // detecting -> tracking
  {
    state = tracking;
    consecutive_detect_frame_cnt_ = 0;
  }

  predict(t_img);

  if (armors.empty()) return;

  for (auto armor : armors) {
    // 装甲板匹配 with debug info
    int id;
    auto min_angle_error = 1e10;
    auto second_min_angle_error = 1e10;
    const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();
    tools::logger()->info("{}'s armor match info:", ARMOR_NAMES[name]);
    for (int i = 0; i < armor_num_; i++) {
      Eigen::Vector3d ypd = tools::xyz2ypd(xyza_list[i].head(3));
      /// TODO: 重写装甲板匹配 cost 函数
      auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza_list[i][3])) +
                         std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));
      tools::logger()->info("  {:.5f}", angle_error);

      if (angle_error < min_angle_error) {
        id = i;
        second_min_angle_error = min_angle_error;
        min_angle_error = angle_error;
      }
    }

    if (id != 0) jumped = true;
    last_id = id;
    tools::logger()->info("==> updating id: {}", id);

    if (second_min_angle_error / min_angle_error < 1.5)
      tools::logger()->warn(
        "### id matching may be wrong, min:{:5f}, second_min:{:5f}", min_angle_error,
        second_min_angle_error);

    update_ypda(armor, id);
  }
}

void Target::update_ypda(const Armor & armor, int id)
{
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
  Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};

  ekf_.update(z, H, R, h, z_subtract);
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }

  return _armor_xyza_list;
}

bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

}  // namespace auto_aim
