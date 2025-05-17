#include "buff_target.hpp"

namespace auto_buff
{
std::atomic<bool> STOP_THREAD(false);
std::atomic<bool> VALID_PARAMS(false);
std::mutex MUTEX;

///voter

Voter::Voter() : clockwise_(0) {}

void Voter::vote(const double angle_last, const double angle_now)
{
  if (std::abs(clockwise_) > 50) return;
  if (angle_last > angle_now)
    clockwise_--;
  else
    clockwise_++;
}

int Voter::clockwise() { return clockwise_ > 0 ? 1 : -1; }

/// Target

Target::Target() : first_in_(true), unsolvable_(true){};

Eigen::Vector3d Target::point_buff2world(const Eigen::Vector3d & point_in_buff) const
{
  if (unsolvable_) return Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix3d R_buff2world =
    tools::rotation_matrix(Eigen::Vector3d(ekf_.x[4], 0.0, ekf_.x[5]));  // pitch = 0

  auto R_yaw = ekf_.x[0];
  auto R_pitch = ekf_.x[2];
  auto R_dis = ekf_.x[3];
  Eigen::Vector3d point_in_world =
    R_buff2world * point_in_buff + Eigen::Vector3d(
                                     R_dis * std::cos(R_pitch) * std::cos(R_yaw),
                                     R_dis * std::cos(R_pitch) * std::sin(R_yaw),
                                     R_dis * std::sin(R_pitch));
  return point_in_world;
}

bool Target::is_unsolve() const { return unsolvable_; }

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

/// SmallTarget

SmallTarget::SmallTarget() : Target() {}

void SmallTarget::get_target(
  const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp)
{
  // 如果没有识别，退出函数
  static int lost_cn = 0;
  if (!p.has_value()) {
    unsolvable_ = true;
    lost_cn++;
    return;
  }

  static std::chrono::steady_clock::time_point start_timestamp = timestamp;
  auto time_gap = tools::delta_time(timestamp, start_timestamp);

  // init
  if (first_in_) {
    unsolvable_ = true;
    init(time_gap, p.value());
    first_in_ = false;
  }

  // 处理识别时间间隔过大
  if (lost_cn > 6) {
    unsolvable_ = true;
    tools::logger()->debug("[Target] 丢失buff");
    lost_cn = 0;
    first_in_ = true;
    return;
  }

  // kalman update
  unsolvable_ = false;
  update(time_gap, p.value());

  // 处理发散
  if (std::abs(ekf_.x[6]) > SMALL_W + CV_PI / 18 || std::abs(ekf_.x[6]) < SMALL_W - CV_PI / 18) {
    unsolvable_ = true;
    tools::logger()->debug("[Target] 小符角度发散spd: {:.2f}", ekf_.x[6] * 180 / CV_PI);
    first_in_ = true;
    return;
  }
}

void SmallTarget::predict(double dt)
{
  // 预测下一个状态
  // clang-format off
  A_ << 1.0,  dt, 0.0, 0.0, 0.0, 0.0, 0.0, // R_yaw
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, // R_v_yaw
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // R_pitch
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, // R_dis
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // yaw
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,  dt, // roll
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0; // spd

  // 过程噪声协方差矩阵                            //// 调整
  auto v1 = 0.9;  // 角加速度方差
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  Q_ << a * v1, b * v1, 0.0, 0.0, 0.0, 0.0, 0.0,
        b * v1, c * v1, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // clang-format on 
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = A_ * x;
    x_prior[0] = tools::limit_rad(x_prior[0]);
    x_prior[2] = tools::limit_rad(x_prior[2]);
    x_prior[4] = tools::limit_rad(x_prior[4]);
    x_prior[5] = tools::limit_rad(x_prior[5]);
    return x_prior;
  };
  ekf_.predict(A_, Q_, f);
}

void SmallTarget::init(double nowtime, const PowerRune & p)
{
  // 初始化内部变量
  lasttime_ = nowtime;

  // 初始状态协方差矩阵
  x0_.resize(7);
  P0_.resize(7, 7);
  A_.resize(7, 7);
  Q_.resize(7, 7);
  H_.resize(7, 7);//z x
  R_.resize(7, 7);//z z
  // [R_yaw]
  // [v_R_yaw]
  // [R_pitch]
  // [R_dis]
  // [yaw]
  // [angle/row]
  // [spd]   w=CV_PI/6

  // clang-format off
  // 初始状态
  x0_ << p.ypd_in_world[0], 0.0, p.ypd_in_world[1], p.ypd_in_world[2],
         p.ypr_in_world[0], p.ypr_in_world[2], 
         SMALL_W * voter.clockwise();
  // 初始状态协方差矩阵
  P0_ << 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 10.0;
  // 状态转移矩阵
  // A_ 
  // 过程噪声协方差矩阵                            //// 调整
  // Q_ 
  // 测量方程矩阵
  // H_
  // 测量噪声协方差矩阵                            //// 调整
  // R_

  // clang-format on

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[0] = tools::limit_rad(c[0]);
    c[2] = tools::limit_rad(c[2]);
    c[4] = tools::limit_rad(c[4]);
    c[5] = tools::limit_rad(c[5]);
    return c;
  };
  // 创建扩展卡尔曼滤波器对象
  ekf_ = tools::ExtendedKalmanFilter(x0_, P0_, x_add);
}

void SmallTarget::update(double nowtime, const PowerRune & p)
{
  // [R_yaw]     angle0
  // [v_R_yaw]
  // [R_pitch]   angle2
  // [R_dis]
  // [yaw]       angle4
  // [angle/row] angle5
  // [spd]   w=CV_PI/6
  const Eigen::VectorXd & R_ypd = p.ypd_in_world;  // R
  const Eigen::VectorXd & ypr = p.ypr_in_world;
  const Eigen::VectorXd & B_ypd = p.blade_ypd_in_world;  // center of blade

  // 处理扇叶跳变 angle/row
  if (abs(ypr[2] - ekf_.x[5]) > CV_PI / 12) {
    for (int i = -5; i <= 5; i++) {
      double angle_c = ekf_.x[5] + i * 2 * CV_PI / 5;
      if (std::fabs(angle_c - ypr[2]) < CV_PI / 5) {
        ekf_.x[5] += i * 2 * CV_PI / 5;
        break;
      }
    }
  }

  // vote判断是顺时针还是逆时针旋转
  voter.vote(ekf_.x[5], ypr[2]);
  if (voter.clockwise() * ekf_.x[6] < 0) ekf_.x[6] *= -1;  // spd

  // 预测下一个状态
  predict(nowtime - lasttime_);

  // [R_yaw]     angle0
  // [R_pitch]   angle1
  // [R_dis]
  // [angle/row] angle3
  // [B_yaw]     angle4
  // [B_pitch]   angle5
  // [B_dis]

  /// 1.

  // [R_yaw]     angle0
  // [R_pitch]   angle1
  // [R_dis]
  // [angle/row] angle3

  // clang-format off
  Eigen::MatrixXd H1{
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // R_yaw
    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}, // R_pitch
    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, // R_dis
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0}  // roll
  };

  Eigen::MatrixXd R1{
    {0.01, 0.0, 0.0,  0.0}, // R_yaw
    {0.0, 0.01, 0.0,  0.0}, // R_pitch
    {0.0,  0.0, 0.5,  0.0}, // R_dis
    {0.0,  0.0, 0.0,  0.01}  // roll
  };
  // clang-format on

  // 防止夹角求差出现异常值
  auto z_subtract1 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;  //4 1
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  Eigen::VectorXd z1{{R_ypd[0], R_ypd[1], R_ypd[2], ypr[2]}};  // R_ypd roll

  ekf_.update(z1, H1, R1, z_subtract1);

  ///2.

  // [B_yaw]     angle4
  // [B_pitch]   angle5
  // [B_dis]

  // clang-format off
  Eigen::MatrixXd H2 = h_jacobian();  // 3*7

  Eigen::MatrixXd R2{
    {0.01, 0.0, 0.0}, // B_yaw
    {0.0, 0.01, 0.0}, // B_pitch
    {0.0,  0.0, 0.5}  // B_dis
  };
  // clang-format on

  // 定义非线性转换函数h: x -> z
  auto h2 = [&](const Eigen::VectorXd & x) -> Eigen::Vector3d {
    Eigen::VectorXd R_ypd{{x[0], x[2], x[3]}};
    Eigen::VectorXd R_xyz = tools::ypd2xyz(R_ypd);
    Eigen::VectorXd R_xyz_and_yr{{R_ypd[0], R_ypd[1], R_ypd[2], x[4], x[5]}};
    Eigen::VectorXd B_xyz = point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
    Eigen::VectorXd B_ypd = tools::xyz2ypd(B_xyz);
    return B_ypd;
  };

  // 防止夹角求差出现异常值
  auto z_subtract2 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;  //6 1
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    return c;
  };

  Eigen::VectorXd z2{{B_ypd[0], B_ypd[1], B_ypd[2]}};

  ekf_.update(z2, H2, R2, h2, z_subtract2);

  // 更新lasttime
  lasttime_ = nowtime;
  return;
}

Eigen::MatrixXd SmallTarget::h_jacobian() const
{
  /// Z(3,1) = H3(3,3) * H2(3,5) * H1(5,5) * H0(5,7) * x(7,1)

  // clang-format off
  Eigen::MatrixXd H0{
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0}
  };// 5*7

  Eigen::VectorXd R_ypd{{ekf_.x[0], ekf_.x[2], ekf_.x[3]}};
  Eigen::MatrixXd H_ypd2xyz = tools::ypd2xyz_jacobian(R_ypd);  // 3*3
  Eigen::MatrixXd H1{
    {H_ypd2xyz(0, 0), H_ypd2xyz(0, 1), H_ypd2xyz(0, 2), 0.0, 0.0},
    {H_ypd2xyz(1, 0), H_ypd2xyz(1, 1), H_ypd2xyz(1, 2), 0.0, 0.0},
    {H_ypd2xyz(2, 0), H_ypd2xyz(2, 1), H_ypd2xyz(2, 2), 0.0, 0.0},
    {            0.0,             0.0,             0.0, 1.0, 0.0},
    {            0.0,             0.0,             0.0, 0.0, 1.0}
  };// 5*5

  // double pitch = 0;
  double yaw = ekf_.x[4];
  double roll = ekf_.x[5];
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  double cos_roll = cos(roll);
  double sin_roll = sin(roll);
  Eigen::MatrixXd H2{
    {1.0, 0.0, 0.0, 0.7 * cos_yaw * sin_roll,  0.7 * sin_yaw * cos_roll},
    {0.0, 1.0, 0.0, 0.7 * sin_yaw * sin_roll, -0.7 * cos_yaw * cos_roll},
    {0.0, 0.0, 1.0,                      0.0,           -0.7 * sin_roll}
  };// 3*5

  Eigen::VectorXd B_xyz = point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  Eigen::MatrixXd H3 = tools::xyz2ypd_jacobian(B_xyz);// 3*3
  // clang-format on

  return H3 * H2 * H1 * H0;  // 3*7

  // auto h2 = [&](const Eigen::VectorXd & x) -> Eigen::Vector3d {
  //   Eigen::VectorXd R_ypd{{x[0], x[2], x[3]}};
  //   Eigen::VectorXd R_xyz = tools::ypd2xyz(R_ypd);
  //   Eigen::VectorXd R_xyz_and_yr{{R_ypd[0], R_ypd[1], R_ypd[2], x[4], x[5]}};
  //   Eigen::VectorXd B_xyz = point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  //   Eigen::VectorXd B_ypd = tools::xyz2ypd(B_xyz);
  //   return B_ypd;
  // };
}

/// BigTarget

BigTarget::BigTarget() : Target() {
  // fit_thread_ = std::thread(&BigTarget::fit, this);
}

BigTarget::BigTarget(const BigTarget& other)
  : Target(other),
    params_(other.params_),
    convexity_(other.convexity_),
    fit_data_(other.fit_data_) {
} 

BigTarget& BigTarget::operator=(const BigTarget& other) {
  if (this != &other) {
    Target::operator=(other);
    params_ = other.params_;
    convexity_ = other.convexity_;
    fit_data_ = other.fit_data_;
  }
  return *this;
}

void BigTarget::get_target(
  const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp)
{
  // 如果没有识别，退出函数
  static int lost_cn = 0;
  if (!p.has_value()) {
    unsolvable_ = true;
    lost_cn++;
    return;
  }

  static std::chrono::steady_clock::time_point start_timestamp = timestamp;
  auto time_gap = tools::delta_time(timestamp, start_timestamp);

  // init
  if (first_in_) {
    unsolvable_ = true;
    init(time_gap, p.value());
    first_in_ = false;
  }

  // 处理识别时间间隔过大
  if (lost_cn > 6) {
    unsolvable_ = true;
    tools::logger()->debug("[Target] 丢失buff");
    lost_cn = 0;
    first_in_ = true;
    return;
  }

  // kalman update
  unsolvable_ = false;
  update(time_gap, p.value());

  // 处理发散
  if (
    ekf_.x[7] > 1.045 * 1.5 || ekf_.x[7] < 0.78 / 1.5 || ekf_.x[8] > 2.0 * 1.5 ||
    ekf_.x[8] < 1.884 / 1.5) {
    tools::logger()->debug("[Target] 大符角度发散a: {:.2f}b:{:.2f}", ekf_.x[7], ekf_.x[8]);
    first_in_ = true;
    return;
  }
}

void BigTarget::get_target_by_fitter(const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp)
{
  // 如果没有识别，退出函数
  static int lost_cn = 0;
  if (!p.has_value()) {
    unsolvable_ = true;
    lost_cn++;
    return;
  }

  static std::chrono::steady_clock::time_point start_timestamp = timestamp;
  auto time_gap = tools::delta_time(timestamp, start_timestamp);

  // init
  if (first_in_) {
    unsolvable_ = true;
    first_in_ = false;
  }

  // 处理识别时间间隔过大
  if (lost_cn > 6) {
    unsolvable_ = true;
    tools::logger()->debug("[Target] 丢失buff");
    lost_cn = 0;
    first_in_ = true;
    return;
  }

  auto power_rune = p.value();
  // 获取拟合数据
  double now_angle = power_rune.ypr_in_world[2] * 57.3;
  voter.vote(last_angle_, now_angle);  // 计算旋转方向
  double delta_angle = now_angle - last_angle_;
  last_angle_ = now_angle;
  int shift =  std::round(delta_angle / 72);
  total_shift_ += shift;
  double delta_angle_rel = now_angle - total_shift_ * 72;
  double time = tools::delta_time(timestamp, start_timestamp);
  
  std::unique_lock lock(mutex_);
  fit_data_.emplace_back(time, std::abs(delta_angle_rel));

}

void BigTarget::predict(double dt)
{
  // 预测下一个状态
  double spd = ekf_.x[6];
  double a = ekf_.x[7];
  double w = ekf_.x[8];
  double fi = ekf_.x[9];
  double t = lasttime_ + dt;
  // clang-format off
  A_ << 1.0,  dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//R_yaw
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//v_R_yaw
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//R_pitch
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,//R_dis
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,//yaw
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, voter.clockwise() * dt , 0.0, 0.0, 0.0,//row
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sin(w * t + fi) - 1, t * a * cos(w * t + fi), a * cos(w * t + fi),//spd
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,//a
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,//w
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;//theta
        
  // 过程噪声协方差矩阵                            //// 调整
  auto v1 = 0.9;  // 角加速度方差
  auto a1 = dt * dt * dt * dt / 4;
  auto b1 = dt * dt * dt / 2;
  auto c1 = dt * dt;
  Q_ << a1 * v1, b1 * v1, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        b1 * v1, c1 * v1, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,     0.0, 0.0, 0.0, 0.0, 0.09,  0.0,  0.0,  0.0,  0.0,//row
            0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.5,  0.0,  0.0,  0.0,// spd 0.5  1
            0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,// a
            0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,// w
            0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  1.0;// fi
            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  1.0,  0.0,  0.0,  0.0,// spd  2
            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 
            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  4.0;

            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 
            // 0.0,     0.0, 0.0, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0;
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = x;
    x_prior[0] = tools::limit_rad(x_prior[0] + dt * x_prior[1]);
    x_prior[2] = tools::limit_rad(x_prior[2]);
    x_prior[4] = tools::limit_rad(x_prior[4]); // yaw
    x_prior[5] = tools::limit_rad(x_prior[5] + voter.clockwise() * 
    (-a / w * std::cos(w * t + fi) + a / w * std::cos(w * lasttime_ + fi) + (2.09 - a) * dt)); // roll
    x_prior[6] = a * sin(w * t + fi) + 2.09 - a; // spd
    return x_prior;
  };
  // clang-format on
  ekf_.predict(A_, Q_, f);
}

void BigTarget::predict_by_fitter(double dt)
{
  
}

void BigTarget::init(double nowtime, const PowerRune & p)
{
  // 初始化内部变量
  lasttime_ = nowtime;
  unsolvable_ = true;

  // 初始状态协方差矩阵
  x0_.resize(10);
  P0_.resize(10, 10);
  A_.resize(10, 10);
  Q_.resize(10, 10);
  H_.resize(7, 10);
  R_.resize(7, 7);

  // [R_yaw]
  // [v_R_yaw]
  // [R_pitch]
  // [R_dis]
  // [yaw]
  // [angle/row]
  // [spd]       角速度 a*sin(wt) + 2.09 - a
  // [a]         0.78-1.045
  // [w]         1.884-2.000
  // [fi]

  // clang-format off
  // 初始状态
  x0_ << p.ypd_in_world[0], 0.0, p.ypd_in_world[1], p.ypd_in_world[2],
         p.ypr_in_world[0], p.ypr_in_world[2], 
         1.1775, 0.9125, 1.942, 0.0;//std::atan((spd - 2.09) / 0.9125 + 1
  // 初始状态协方差矩阵
  P0_ << 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 100.0, 0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 400.0;
  // 状态转移矩阵
  // A_
  // 过程噪声协方差矩阵                            //// 调整
  // Q_
  // 测量方程矩阵
  // H_
  // 测量噪声协方差矩阵                            //// 调整
  // R_

  // clang-format on

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[0] = tools::limit_rad(c[0]);
    c[2] = tools::limit_rad(c[2]);
    c[4] = tools::limit_rad(c[4]);
    c[5] = tools::limit_rad(c[5]);
    c[9] = tools::limit_rad(c[9]);
    return c;
  };
  // 创建扩展卡尔曼滤波器对象
  ekf_ = tools::ExtendedKalmanFilter(x0_, P0_, x_add);
}

void BigTarget::update(double nowtime, const PowerRune & p)
{
  // [R_yaw]
  // [v_R_yaw]
  // [R_pitch]
  // [R_dis]
  // [yaw]
  // [angle/row] 角度
  // [spd]       角速度 a*sin(wt) + 2.09 - a
  // [a]         0.78-1.045
  // [w]         1.884-2.000
  // [fi]
  const Eigen::VectorXd & R_ypd = p.ypd_in_world;  // R
  const Eigen::VectorXd & ypr = p.ypr_in_world;
  const Eigen::VectorXd & B_ypd = p.blade_ypd_in_world;  // center of blade

  // 处理扇叶跳变 angle/row
  if (abs(ypr[2] - ekf_.x[5]) > CV_PI / 12) {
    for (int i = -5; i <= 5; i++) {
      double angle_c = ekf_.x[5] + i * 2 * CV_PI / 5;
      if (std::fabs(angle_c - ypr[2]) < CV_PI / 5) {
        ekf_.x[5] += i * 2 * CV_PI / 5;
        break;
      }
    }
  }

  // vote判断是顺时针还是逆时针旋转
  voter.vote(ekf_.x[5], ypr[2]);

  auto anglelast = ekf_.x[5];  ///

  // 预测下一个状态
  predict(nowtime - lasttime_);

  // [R_yaw]     angle0
  // [R_pitch]   angle1
  // [R_dis]
  // [angle/row] angle3
  // [B_yaw]     angle4
  // [B_pitch]   angle5
  // [B_dis]

  /// 1.

  // [R_yaw]     angle0
  // [R_pitch]   angle1
  // [R_dis]
  // [angle/row] angle3

  // clang-format off
  Eigen::MatrixXd H1{
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // R_yaw
    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // R_pitch
    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // R_dis
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}  // roll
  };

  Eigen::MatrixXd R1{
    {0.01, 0.0, 0.0,  0.0}, // R_yaw
    {0.0, 0.01, 0.0,  0.0}, // R_pitch
    {0.0,  0.0, 0.5,  0.0}, // R_dis
    {0.0,  0.0, 0.0, 0.01}  // roll  1: 0.01 2:0.04
  };
  // clang-format on

  // 防止夹角求差出现异常值
  auto z_subtract1 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;  //4 1
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  Eigen::VectorXd z1{{R_ypd[0], R_ypd[1], R_ypd[2], ypr[2]}};  // R_ypd roll

  ekf_.update(z1, H1, R1, z_subtract1);

  ///2.

  // [B_yaw]     angle4
  // [B_pitch]   angle5
  // [B_dis]

  // clang-format off
  Eigen::MatrixXd H2 = h_jacobian();  // 3*10

  Eigen::MatrixXd R2{
    {0.01, 0.0, 0.0}, // B_yaw
    {0.0, 0.01, 0.0}, // B_pitch
    {0.0,  0.0, 0.5}  // B_dis
  };
  // clang-format on

  // 定义非线性转换函数h: x -> z
  auto h2 = [&](const Eigen::VectorXd & x) -> Eigen::Vector3d {
    Eigen::VectorXd R_ypd{{x[0], x[2], x[3]}};
    Eigen::VectorXd R_xyz = tools::ypd2xyz(R_ypd);
    Eigen::VectorXd R_xyz_and_yr{{R_ypd[0], R_ypd[1], R_ypd[2], x[4], x[5]}};
    Eigen::VectorXd B_xyz = point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
    Eigen::VectorXd B_ypd = tools::xyz2ypd(B_xyz);
    return B_ypd;
  };

  // 防止夹角求差出现异常值
  auto z_subtract2 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;  //6 1
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    return c;
  };

  Eigen::VectorXd z2{{B_ypd[0], B_ypd[1], B_ypd[2]}};

  ekf_.update(z2, H2, R2, h2, z_subtract2);

  spd = voter.clockwise() * (ekf_.x[5] - anglelast) / (nowtime - lasttime_);  ///
  if (std::abs(spd) > 4) spd = 0;                                             ///

  // 更新lasttime
  lasttime_ = nowtime;
  unsolvable_ = false;
  return;
}

Eigen::MatrixXd BigTarget::h_jacobian() const
{
  /// Z(3,1) = H3(3,3) * H2(3,5) * H1(5,5) * H0(5,10) * x(10,1)

  // clang-format off
  Eigen::MatrixXd H0{
    {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}
  };// 5*7

  Eigen::VectorXd R_ypd{{ekf_.x[0], ekf_.x[2], ekf_.x[3]}};
  Eigen::MatrixXd H_ypd2xyz = tools::ypd2xyz_jacobian(R_ypd);  // 3*3
  Eigen::MatrixXd H1{
    {H_ypd2xyz(0, 0), H_ypd2xyz(0, 1), H_ypd2xyz(0, 2), 0.0, 0.0},
    {H_ypd2xyz(1, 0), H_ypd2xyz(1, 1), H_ypd2xyz(1, 2), 0.0, 0.0},
    {H_ypd2xyz(2, 0), H_ypd2xyz(2, 1), H_ypd2xyz(2, 2), 0.0, 0.0},
    {            0.0,             0.0,             0.0, 1.0, 0.0},
    {            0.0,             0.0,             0.0, 0.0, 1.0}
  };// 5*5

  // double pitch = 0;
  double yaw = ekf_.x[4];
  double roll = ekf_.x[5];
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  double cos_roll = cos(roll);
  double sin_roll = sin(roll);
  Eigen::MatrixXd H2{
    {1.0, 0.0, 0.0, 0.7 * cos_yaw * sin_roll,  0.7 * sin_yaw * cos_roll},
    {0.0, 1.0, 0.0, 0.7 * sin_yaw * sin_roll, -0.7 * cos_yaw * cos_roll},
    {0.0, 0.0, 1.0,                      0.0,           -0.7 * sin_roll}
  };// 3*5

  Eigen::VectorXd B_xyz = point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  Eigen::MatrixXd H3 = tools::xyz2ypd_jacobian(B_xyz);// 3*3
  // clang-format on

  return H3 * H2 * H1 * H0;  // 3*7

  // auto h2 = [&](const Eigen::VectorXd & x) -> Eigen::Vector3d {
  //   Eigen::VectorXd R_ypd{{x[0], x[2], x[3]}};
  //   Eigen::VectorXd R_xyz = tools::ypd2xyz(R_ypd);
  //   Eigen::VectorXd R_xyz_and_yr{{R_ypd[0], R_ypd[1], R_ypd[2], x[4], x[5]}};
  //   Eigen::VectorXd B_xyz = point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  //   Eigen::VectorXd B_ypd = tools::xyz2ypd(B_xyz);
  //   return B_ypd;
  // };
}

/**
 * @brief 拟合一次
 */
bool BigTarget::fitOnce() {
  // 如果数据量过少，则确定凹凸性
  if (fit_data_.size() < (size_t)2 * MIN_FIT_DATA_SIZE) {
      convexity_ = getConvexity(fit_data_);
  }
  // 利用 ransac 算法计算参数
  params_ = ransacFitting(fit_data_, convexity_);
  return true;
}

/**
* @brief 拟合线程的主函数
*/
void BigTarget::fit() {
  decltype(fit_data_) fitData;
  while (STOP_THREAD.load() == false) {
    {
      std::shared_lock lock(mutex_);
      fitData = fit_data_;
    }
    // 数据量过少时，直接返回
    if (fit_data_.size() < (size_t)MIN_FIT_DATA_SIZE) {
      continue;
    }
    bool result = fitOnce();
    VALID_PARAMS.store(result);
    if(debug_) {
      MUTEX.lock();
      if (result == true) {
        std::cout << "params: ";
        std::for_each(params_.begin(), params_.end(), [](auto &&it) { std::cout << it << " "; });
        std::cout << std::endl;
      }
      MUTEX.unlock();
    }
    if (fit_data_.size() > (size_t)MAX_FIT_DATA_SIZE) {
      fit_data_.erase(fit_data_.begin(), fit_data_.begin() + fit_data_.size() / 2);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(int(1e4 / FPS)));
  }
}


/**
 * @brief 凹凸性计算
 * @param[in] data          角度数据
 * @return Convexity
 */
Convexity getConvexity(const std::vector<std::pair<double, double>> &data) {
  auto first{data.begin()}, last{data.end() - 1};
  double slope{(last->second - first->second) / (last->first - first->first)};
  double offset{(first->second * last->first - last->second * first->first) / (last->first - first->first)};
  int concave{0}, convex{0};
  for (const auto &i : data) {
      if (slope * i.first + offset > i.second) {
          concave++;
      } else {
          convex++;
      }
  }
  const int standard{static_cast<int>(data.size() * 0.75)};
  return concave > standard  ? Convexity::CONCAVE
         : convex > standard ? Convexity::CONVEX
                             : Convexity::UNKNOWN;
}

/**
* @brief ransac 算法，返回拟合参数
* @param[in] data          角度数据
* @param[in] convexity     凹凸性
* @return std::array<double, 5>
*/
std::array<double, 5> ransacFitting(const std::vector<std::pair<double, double>> &data, Convexity convexity) {
  // inliers 为符合要求的点，outliers 为不符合要求的点
  std::vector<std::pair<double, double>> inliers, outliers;
  // 初始时，inliers 为全部点
  inliers.assign(data.begin(), data.end());
  // 迭代次数
  int iterTimes{data.size() < 400 ? 200 : 20};
  // 初始参数
  std::array<double, 5> params{0.470, 1.942, 0, 1.178, 0};
  for (int i = 0; i < iterTimes; ++i) {
      decltype(inliers) sample;
      // 如果数据点较多，则将数据打乱，取其中一部分
      if (inliers.size() > 400) {
          std::shuffle(
              inliers.begin(), inliers.end() - 100,
              std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count()));
          sample.assign(inliers.end() - 200, inliers.end());
      } else {
          sample.assign(inliers.begin(), inliers.end());
      }
      // 进行拟合
      params = leastSquareEstimate(sample, params, convexity);
      // 对 inliers 每一个计算误差
      std::vector<double> errors;
      for (const auto &inlier : inliers) {
          errors.push_back(std::abs(inlier.second - getAngleBig(inlier.first, params)));
      }
      // 如果数据量较大，则对点进行筛选
      if (data.size() > 800) {
          std::sort(errors.begin(), errors.end());
          const int index{static_cast<int>(errors.size() * 0.95)};
          const double threshold{errors[index]};
          // 剔除 inliers 中不符合要求的点
          for (size_t i = 0; i < inliers.size() - 100; ++i) {
              if (std::abs(inliers[i].second - getAngleBig(inliers[i].first, params)) > threshold) {
                  outliers.push_back(inliers[i]);
                  inliers.erase(inliers.begin() + i);
              }
          }
          // 将 outliers 中符合要求的点加进来
          for (size_t i = 0; i < outliers.size(); ++i) {
              if (std::abs(outliers[i].second - getAngleBig(outliers[i].first, params)) < threshold) {
                  inliers.emplace(inliers.begin(), outliers[i]);
                  outliers.erase(outliers.begin() + i);
              }
          }
      }
  }
  // 返回之前对所有 inliers 再拟合一次
  return params;
}

/**
* @brief 最小二乘拟合，返回参数列表
* @param[in] points        数据点
* @param[in] params        初始参数
* @param[in] convexity     凹凸性
* @return std::array<double, 5>
*/
std::array<double, 5> leastSquareEstimate(const std::vector<std::pair<double, double>> &points,
                                        const std::array<double, 5> &params, Convexity convexity) {
  std::array<double, 5> ret = params;
  ceres::Problem problem;
  for (size_t i = 0; i < points.size(); i++) {
      ceres::CostFunction *costFunction = new CostFunctor2(points[i].first, points[i].second);
      ceres::LossFunction *lossFunction = new ceres::SoftLOneLoss(0.1);
      problem.AddResidualBlock(costFunction, lossFunction, ret.begin());
  }
  std::array<double, 3> omega;
  if (points.size() < 100) {
      // 在数据量较小时，可以利用凹凸性定参数边界
      if (convexity == Convexity::CONCAVE) {
          problem.SetParameterUpperBound(ret.begin(), 2, -2.8);
          problem.SetParameterLowerBound(ret.begin(), 2, -4);
      } else if (convexity == Convexity::CONVEX) {
          problem.SetParameterUpperBound(ret.begin(), 2, -1.1);
          problem.SetParameterLowerBound(ret.begin(), 2, -2.3);
      }
      omega = {10., 1., 1.};
  } else {
      // 而数据量较多后，则不再需要凹凸性辅助拟合
      omega = {60., 50., 50.};
  }
  ceres::CostFunction *costFunction1 = new CostFunctor1(ret[0], 0);
  ceres::LossFunction *lossFunction1 =
      new ceres::ScaledLoss(new ceres::HuberLoss(0.1), omega[0], ceres::TAKE_OWNERSHIP);
  problem.AddResidualBlock(costFunction1, lossFunction1, ret.begin());
  ceres::CostFunction *costFunction2 = new CostFunctor1(ret[1], 1);
  ceres::LossFunction *lossFunction2 =
      new ceres::ScaledLoss(new ceres::HuberLoss(0.1), omega[1], ceres::TAKE_OWNERSHIP);
  problem.AddResidualBlock(costFunction2, lossFunction2, ret.begin());
  ceres::CostFunction *costFunction3 = new CostFunctor1(ret[3], 3);
  ceres::LossFunction *lossFunction3 =
      new ceres::ScaledLoss(new ceres::HuberLoss(0.1), omega[2], ceres::TAKE_OWNERSHIP);
  problem.AddResidualBlock(costFunction3, lossFunction3, ret.begin());
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 50;
  options.minimizer_progress_to_stdout = false;
  options.check_gradients = false;
  options.gradient_check_relative_precision = 1e-4;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  return ret;
}

}  // namespace auto_buff