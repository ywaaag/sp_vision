#include "ekfpnp.hpp"

#include <yaml-cpp/yaml.h>

#include <iostream>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
EKFPnP::EKFPnP(const std::string & config_path) : is_initialized_(false)
{
  auto yaml = YAML::LoadFile(config_path);

  // 初始化 P0_
  auto p0_data = yaml["P0"].as<std::vector<double>>();
  if (p0_data.size() != 13) throw std::runtime_error("P0 数据尺寸错误，应为 13");
  P0_ = Eigen::VectorXd::Map(p0_data.data(), p0_data.size());
  P_ = Eigen::MatrixXd::Zero(13, 13);
  for (int i = 0; i < 13; ++i) P_(i, i) = P0_(i);

  // 初始化 V0_
  auto v0_data = yaml["V0"].as<std::vector<double>>();
  if (v0_data.size() != 3) throw std::runtime_error("V0 数据尺寸错误，应为 3");
  V0_ = Eigen::Map<const Eigen::Vector3d>(v0_data.data());

  // 初始化 Omega0_
  auto omega0_data = yaml["Omega0"].as<std::vector<double>>();
  if (omega0_data.size() != 3) throw std::runtime_error("Omega0 数据尺寸错误，应为 3");
  Omega0_ = Eigen::Map<const Eigen::Vector3d>(omega0_data.data());

  // 初始化 Q0_
  auto q0_data = yaml["Q0"].as<std::vector<double>>();
  if (q0_data.size() != 6) throw std::runtime_error("Q0 数据尺寸错误，应为 6");
  Q0_ = Eigen::MatrixXd::Zero(6, 6);
  for (int i = 0; i < 6; ++i) Q0_(i, i) = q0_data[i];

  // 初始化相机内参
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  if (camera_matrix_data.size() != 9)
    throw std::runtime_error("camera_matrix 数据尺寸错误，应为 9");
  K_ << camera_matrix_data[0], camera_matrix_data[4], camera_matrix_data[2], camera_matrix_data[5];

  linear_a_noise_ << 2.2204e-16, 2.2204e-16, 2.2204e-16;
  angular_a_noise_ << 2.2204e-16, 2.2204e-16, 2.2204e-16;

  std::cout << "Camera parameters K_ (fx, fy, cx, cy): " << K_.transpose() << std::endl;
}

void EKFPnP::init_EKFPnP(
  const Eigen::Vector3d & c0, const Eigen::Quaterniond q0, std::chrono::steady_clock::time_point t)
{
  X_.resize(13);
  X_ << c0[0], c0[1], c0[2], q0.w(), q0.x(), q0.y(), q0.z(), V0_[0], V0_[1], V0_[2], Omega0_[0],
    Omega0_[1], Omega0_[2];
  for (int i = 0; i < 13; ++i) P_(i, i) = P0_(i);
  t_ = t;
  is_initialized_ = true;
  std::cout << "initial xyz_in_camera:" << std::endl;
  std::cout << -(q0.toRotationMatrix().inverse() * c0) << std::endl;
  std::cout << "initial P:" << std::endl;
  std::cout << P_.diagonal() << std::endl;
}

void EKFPnP::deinit_EKFPnP()
{
  X_.setZero();
  P_.setZero();
  is_initialized_ = false;
}

void EKFPnP::predict(std::chrono::steady_clock::time_point t)
{
  if (!is_initialized_) {
    t_ = t;
    return;
  }
  auto dt = tools::delta_time(t, t_);
  tools::logger()->debug("dt is {:.4f}", dt);
  t_ = t;

  // 状态转移函数
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior(13);

    x_prior[0] = x[0] + (x[7] + linear_a_noise_[0]) * dt;  //x
    x_prior[1] = x[1] + (x[8] + linear_a_noise_[1]) * dt;  //y
    x_prior[2] = x[2] + (x[9] + linear_a_noise_[2]) * dt;  //z

    Eigen::Quaterniond q1{x[3], x[4], x[5], x[6]};
    Eigen::Vector3d rvc{{x[10], x[11], x[12]}};
    Eigen::Vector3d rotation_vector = (rvc + angular_a_noise_) * dt;

    double angle = rotation_vector.norm();                // 计算旋转角度（模长）
    Eigen::Vector3d axis = rotation_vector.normalized();  // 计算单位方向向量
    // 构造 Eigen::AngleAxisd
    Eigen::AngleAxisd rotation(angle, axis);

    Eigen::Quaterniond q2(rotation);  //旋转向量转四元数
    Eigen::Quaterniond q_prior = q1 * q2;
    x_prior[3] = q_prior.w();  //q.w
    x_prior[4] = q_prior.x();  //q.x
    x_prior[5] = q_prior.y();  //q.y
    x_prior[6] = q_prior.z();  //q.z

    x_prior[7] += linear_a_noise_[0];  //vx
    x_prior[8] += linear_a_noise_[1];  //vy
    x_prior[9] += linear_a_noise_[2];  //vz

    x_prior[10] += angular_a_noise_[0];  //wx
    x_prior[11] += angular_a_noise_[1];  //wy
    x_prior[12] += angular_a_noise_[2];  //wz

    return x_prior;
  };

  // 状态预测
  X_ = f(X_);

  Eigen::Vector3d rotate_vector{X_[10], X_[11], X_[12]};
  rotate_vector *= dt;

  Eigen::Quaterniond qwt = v2q(rotate_vector);
  Eigen::Quaterniond q(X_[3], X_[4], X_[5], X_[6]);

  Eigen::Matrix<double, 4, 3> dq3_by_omega =
    dq3_by_dq1(q) * dqomegadt_by_domega({X_[10], X_[11], X_[12]}, dt);

  // clang-format off

  // 状态转移矩阵的雅可比 13x13
  Eigen::MatrixXd F{
    {1,  0,  0,        0,        0,        0,        0, dt,  0,  0,                  0,                  0,                  0},//x
    {0,  1,  0,        0,        0,        0,        0,  0, dt,  0,                  0,                  0,                  0},//y
    {0,  0,  1,        0,        0,        0,        0,  0,  0, dt,                  0,                  0,                  0},//z
    {0,  0,  0,  qwt.w(), -qwt.x(), -qwt.y(), -qwt.z(),  0,  0,  0,  dq3_by_omega(0,0),  dq3_by_omega(0,1),  dq3_by_omega(0,2)},//q.w
    {0,  0,  0,  qwt.x(),  qwt.w(),  qwt.z(), -qwt.y(),  0,  0,  0,  dq3_by_omega(1,0),  dq3_by_omega(1,1),  dq3_by_omega(1,2)},//q.x
    {0,  0,  0,  qwt.y(), -qwt.z(),  qwt.w(),  qwt.x(),  0,  0,  0,  dq3_by_omega(2,0),  dq3_by_omega(2,1),  dq3_by_omega(2,2)},//q.y
    {0,  0,  0,  qwt.z(),  qwt.y(), -qwt.x(),  qwt.w(),  0,  0,  0,  dq3_by_omega(3,0),  dq3_by_omega(3,1),  dq3_by_omega(3,2)},//q.z
    {0,  0,  0,        0,        0,        0,        0,  1,  0,  0,                  0,                  0,                  0},//vx
    {0,  0,  0,        0,        0,        0,        0,  0,  1,  0,                  0,                  0,                  0},//vy
    {0,  0,  0,        0,        0,        0,        0,  0,  0,  1,                  0,                  0,                  0},//vz
    {0,  0,  0,        0,        0,        0,        0,  0,  0,  0,                  1,                  0,                  0},//wx
    {0,  0,  0,        0,        0,        0,        0,  0,  0,  0,                  0,                  1,                  0},//wy
    {0,  0,  0,        0,        0,        0,        0,  0,  0,  0,                  0,                  0,                  1} //wz(yaw)
  };

  // 控制输入的雅可比 13x6
  Eigen::MatrixXd G{
    {dt,  0,  0,                  0,                  0,                 0,},
    { 0, dt,  0,                  0,                  0,                 0,},
    { 0,  0, dt,                  0,                  0,                 0,},
    { 0,  0,  0,  dq3_by_omega(0,0),  dq3_by_omega(0,1),  dq3_by_omega(0,2)},
    { 0,  0,  0,  dq3_by_omega(1,0),  dq3_by_omega(1,1),  dq3_by_omega(1,2)},
    { 0,  0,  0,  dq3_by_omega(2,0),  dq3_by_omega(2,1),  dq3_by_omega(2,2)},
    { 0,  0,  0,  dq3_by_omega(3,0),  dq3_by_omega(3,1),  dq3_by_omega(3,2)},
    { 1,  0,  0,                  0,                  0,                 0,},
    { 0,  1,  0,                  0,                  0,                 0,},
    { 0,  0,  1,                  0,                  0,                 0,},
    { 0,  0,  0,                  1,                  0,                 0,},
    { 0,  0,  0,                  0,                  1,                 0,},
    { 0,  0,  0,                  0,                  0,                 1,}
  };
  // clang-format on

  // 更新协方差矩阵 13x13
  P_ = F * P_ * F.transpose() + G * Q0_ * G.transpose();

  // debug 输出
  Eigen::Vector4d wxyz = X_.segment<4>(3);
  Eigen::Quaterniond debugq(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
  std::cout << "after predict xyz_in_camera is :" << std::endl;
  std::cout << -(debugq.toRotationMatrix().inverse() * X_.head<3>()) << std::endl;
  // std::cout << "after predict P is :" << std::endl;
  // std::cout << P_ << std::endl;
}

void EKFPnP::update(
  const std::vector<cv::Point3f> & cvpoints_in_world,
  const std::vector<cv::Point2f> & cvpoints_in_pixel)
{
  // clang-format off
  // 观测向量 8x1
  Eigen::VectorXd z{
    {cvpoints_in_pixel[0].x, cvpoints_in_pixel[0].y, cvpoints_in_pixel[1].x, cvpoints_in_pixel[1].y,
     cvpoints_in_pixel[2].x, cvpoints_in_pixel[2].y, cvpoints_in_pixel[3].x, cvpoints_in_pixel[3].y
    }};
  // clang-format on

  Eigen::Vector3d C = X_.head<3>();                   //待估计tvc
  Eigen::Quaterniond q1{X_[3], X_[4], X_[5], X_[6]};  //待估计rvc

  Eigen::Matrix<double, 3, 4> points_in_world;
  for (int i = 0; i < 4; ++i) {
    points_in_world(0, i) = cvpoints_in_world[i].x;
    points_in_world(1, i) = cvpoints_in_world[i].y;
    points_in_world(2, i) = cvpoints_in_world[i].z;
  }

  Eigen::Matrix2d f_matrix = K_.head<2>().asDiagonal();
  Eigen::Vector2d c = K_.tail<2>();  //偏移

  Eigen::MatrixXd v(4, 4);
  v.row(0).setZero();
  v.bottomRows(3) = points_in_world.colwise() - C;

  Eigen::Matrix<double, 3, 4> points_in_camera;
  for (int i = 0; i < 4; ++i) {
    Eigen::Vector3d Pw = points_in_world.col(i);
    Eigen::Vector3d Pc = q1.inverse() * (Pw - C);
    points_in_camera.col(i) = Pc;
  }

  Eigen::Matrix<double, 2, 4> points_in_pixel;
  for (int i = 0; i < 4; ++i) {
    points_in_pixel.col(i) = f_matrix * points_in_camera.col(i).head<2>() / points_in_camera(2, i);
  }

  points_in_pixel.colwise() += c;

  // 雅可比矩阵 8x13
  Eigen::MatrixXd H = dfh_by_ds(X_, points_in_world, points_in_camera, K_);

  // 重投影向量 8x1
  Eigen::VectorXd points_in_pixel_vec =
    Eigen::Map<const Eigen::VectorXd>(points_in_pixel.data(), points_in_pixel.size());

  Eigen::VectorXd inv_R_D = R_.diagonal().array().inverse();  // 取对角线并求逆
  Eigen::MatrixXd B = P_ * H.transpose();                     //13x8

  // 使用 broadcasting 方式调整 B 的列
  for (int i = 0; i < 8; ++i) {
    B.col(i) *= inv_R_D(i);
  }

  Eigen::MatrixXd tmp = B * H;  // 13x13
  K_gain_ = B - tmp * ((Eigen::MatrixXd::Identity(13, 13) + tmp).inverse()) * B;

  // 更新状态向量
  X_ = X_ + K_gain_ * (z - points_in_pixel_vec);
  P_ = (Eigen::MatrixXd::Identity(13, 13) - K_gain_ * H) * P_;

  Eigen::Quaterniond q(X_.segment<4>(3));  // 提取四元数部分
  Eigen::Matrix4d Jn = normJac(q);
  q.normalize();  // 归一化四元数
  X_.segment<4>(3) = q.coeffs();

  // 更新协方差矩阵，按照四元数调整
  P_.block<3, 3>(0, 0) = P_.block<3, 3>(0, 0);
  P_.block<3, 4>(0, 3).setZero();
  P_.block<4, 3>(3, 0).setZero();
  P_.block<4, 4>(3, 3) = Jn * P_.block<4, 4>(3, 3) * Jn.transpose();
  P_.block<3, 3>(7, 7) = P_.block<3, 3>(7, 7);

  Eigen::Vector4d wxyz = X_.segment<4>(3);
  Eigen::Quaterniond debugq(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
  std::cout << "after update xyz_in_camera is :" << std::endl;
  std::cout << -(debugq.toRotationMatrix().inverse() * X_.head<3>()) << std::endl;
  // std::cout << "after update P is :" << std::endl;
  // std::cout << P_ << std::endl;
}

void EKFPnP::iterate_EKFPnP(
  Eigen::Vector3d & xyz_in_camera, Eigen::Matrix3d & R_armor2camera,
  std::chrono::steady_clock::time_point t, const std::vector<cv::Point3f> & cvpoints_in_world,
  const std::vector<cv::Point2f> & cvpoints_in_pixel)
{
  this->predict(t);
  this->update(cvpoints_in_world, cvpoints_in_pixel);
  Eigen::Quaterniond q_camera2armor{X_[3], X_[4], X_[5], X_[6]};
  Eigen::Vector3d camera_in_xyz{{X_[0], X_[1], X_[2]}};
  R_armor2camera = q_camera2armor.toRotationMatrix();
  xyz_in_camera = -R_armor2camera * camera_in_xyz;
}

Eigen::Matrix<double, 4, 3> EKFPnP::dqomegadt_by_domega(const Eigen::Vector3d & omega, double dt)
{
  Eigen::Matrix<double, 4, 3> dqomegadt_by_domegaRES;

  double omegamod = omega.norm();  // 角速度的模长

  dqomegadt_by_domegaRES(0, 0) = dq0_by_domegaA(omega(0), omegamod, dt);
  dqomegadt_by_domegaRES(0, 1) = dq0_by_domegaA(omega(1), omegamod, dt);
  dqomegadt_by_domegaRES(0, 2) = dq0_by_domegaA(omega(2), omegamod, dt);

  dqomegadt_by_domegaRES(1, 0) = dqA_by_domegaA(omega(0), omegamod, dt);
  dqomegadt_by_domegaRES(1, 1) = dqA_by_domegaB(omega(0), omega(1), omegamod, dt);
  dqomegadt_by_domegaRES(1, 2) = dqA_by_domegaB(omega(0), omega(2), omegamod, dt);

  dqomegadt_by_domegaRES(2, 0) = dqA_by_domegaB(omega(1), omega(0), omegamod, dt);
  dqomegadt_by_domegaRES(2, 1) = dqA_by_domegaA(omega(1), omegamod, dt);
  dqomegadt_by_domegaRES(2, 2) = dqA_by_domegaB(omega(1), omega(2), omegamod, dt);

  dqomegadt_by_domegaRES(3, 0) = dqA_by_domegaB(omega(2), omega(0), omegamod, dt);
  dqomegadt_by_domegaRES(3, 1) = dqA_by_domegaB(omega(2), omega(1), omegamod, dt);
  dqomegadt_by_domegaRES(3, 2) = dqA_by_domegaA(omega(2), omegamod, dt);

  return dqomegadt_by_domegaRES;
}

// 辅助函数：计算雅可比矩阵的各部分
double EKFPnP::dq0_by_domegaA(double omegaA, double omega, double dt)
{
  return (-dt / 2.0) * (omegaA / omega) * std::sin(omega * dt / 2.0);
}

double EKFPnP::dqA_by_domegaA(double omegaA, double omega, double dt)
{
  return (dt / 2.0) * std::pow(omegaA, 2) / std::pow(omega, 2) * std::cos(omega * dt / 2.0) +
         (1.0 / omega) * (1.0 - std::pow(omegaA, 2) / std::pow(omega, 2)) *
           std::sin(omega * dt / 2.0);
}

double EKFPnP::dqA_by_domegaB(double omegaA, double omegaB, double omega, double dt)
{
  return (omegaA * omegaB / std::pow(omega, 2)) *
         ((dt / 2.0) * std::cos(omega * dt / 2.0) - (1.0 / omega) * std::sin(omega * dt / 2.0));
}

Eigen::Matrix4d EKFPnP::dq3_by_dq1(const Eigen::Quaterniond & q)
{
  Eigen::Matrix4d dq3_by_dq1{
    {q.w(), -q.x(), -q.y(), -q.z()},
    {q.x(), q.w(), -q.z(), q.y()},
    {q.y(), q.z(), q.w(), -q.x()},
    {q.z(), -q.y(), q.x(), q.w()}};
  return dq3_by_dq1;
}

// 规范化四元数的雅可比矩阵
Eigen::Matrix4d EKFPnP::normJac(const Eigen::Quaterniond & q)
{
  // 提取四元数的分量
  double r = q.w();
  double x = q.x();
  double y = q.y();
  double z = q.z();

  // 计算归一化因子
  double normFactor = pow(r * r + x * x + y * y + z * z, -1.5);

  Eigen::Matrix4d J;

  J << (x * x + y * y + z * z) * normFactor, -r * x * normFactor, -r * y * normFactor,
    -r * z * normFactor, -x * r * normFactor, (r * r + y * y + z * z) * normFactor,
    -x * y * normFactor, -x * z * normFactor, -y * r * normFactor, -y * x * normFactor,
    (r * r + x * x + z * z) * normFactor, -y * z * normFactor, -z * r * normFactor,
    -z * x * normFactor, -z * y * normFactor, (r * r + x * x + y * y) * normFactor;

  return J;
}

// 计算观测矩阵的雅可比矩阵
Eigen::MatrixXd EKFPnP::dfh_by_ds(
  const Eigen::Matrix<double, 13, 1> & s, const Eigen::Matrix<double, 3, 4> & X,
  const Eigen::Matrix<double, 3, 4> & X_C, const Eigen::Vector4d & dist)
{
  Eigen::Vector3d C = s.segment<3>(0);           // 相机位置 C
  Eigen::Quaterniond q(s(3), s(4), s(5), s(6));  // 四元数 q (qr, qv)
  double f = dist(0);                            // 相机焦距 (TODO需要考虑fx、fy)
  int n = X_C.cols();                            // 3D 点的数量

  // 初始化雅可比矩阵 H，大小为 2*n x 13
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2 * n, 13);

  // --- 计算 d(h_CI)/d(H_WC)
  Eigen::MatrixXd dhci_by_dhwc = Eigen::MatrixXd::Zero(2 * n, 3);
  for (int i = 0; i < n; ++i) {
    double inv_z = 1.0 / X_C(2, i);
    dhci_by_dhwc(2 * i, 0) = f * inv_z;
    dhci_by_dhwc(2 * i, 2) = -f * X_C(0, i) * inv_z * inv_z;
    dhci_by_dhwc(2 * i + 1, 1) = f * inv_z;
    dhci_by_dhwc(2 * i + 1, 2) = -f * X_C(1, i) * inv_z * inv_z;
  }

  // --- 计算 d(H_WC)/ds
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Vector3d qv = q.vec();  // q 的虚部
  double qr = q.w();             // q 的实部

  // 使用反对称矩阵替代叉积
  Eigen::Matrix3d skew_qv;
  skew_qv << 0, -qv(2), qv(1), qv(2), 0, -qv(0), -qv(1), qv(0), 0;

  Eigen::Matrix3d dhwc_by_dc =
    -(qr * qr - qv.squaredNorm()) * I - 2 * qv * qv.transpose() + 2 * qr * skew_qv;

  Eigen::MatrixXd v = X.colwise() - C;
  Eigen::MatrixXd qv_rep = qv.replicate(1, n);
  Eigen::VectorXd dot_v_qv = (qv_rep.array() * v.array()).colwise().sum();

  // 对每一列计算叉积，确保符合 Eigen 语法
  Eigen::MatrixXd crs_v_qv(3, n);
  for (int i = 0; i < n; ++i) {
    crs_v_qv.col(i) = qr * v.col(i) + skew_qv * v.col(i);
  }

  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d vi = v.col(i);
    Eigen::Matrix3d skew_vi;
    skew_vi << 0, -vi(2), vi(1), vi(2), 0, -vi(0), -vi(1), vi(0), 0;

    Eigen::Matrix3d dwc_by_dq =
      2 * (dot_v_qv(i) * I + qv * vi.transpose() - vi * qv.transpose() - qr * skew_vi);

    // 将雅可比矩阵组合起来
    Eigen::MatrixXd dhh_by_dx_m(3, 13);
    dhh_by_dx_m.block<3, 3>(0, 0) = dhwc_by_dc;
    dhh_by_dx_m.block<3, 3>(0, 3) = dwc_by_dq;
    dhh_by_dx_m.block<3, 6>(0, 6).setZero();  // 其余部分填充 0

    Eigen::MatrixXd dhci_by_dhwc_m = dhci_by_dhwc.block<2, 3>(2 * i, 0);
    H.block<2, 13>(2 * i, 0) = dhci_by_dhwc_m * dhh_by_dx_m;
  }

  return H;
}

// 旋转向量转四元数
Eigen::Quaterniond EKFPnP::v2q(const Eigen::Vector3d & v)
{
  // // 计算旋转向量的模长
  double nrm = v.norm();

  // 如果模长接近0，返回单位四元数
  if (nrm < 2.2204e-16) {
    return Eigen::Quaterniond(1, 0, 0, 0);  // 单位四元数
  } else {
    // 归一化旋转向量
    Eigen::Vector3d v_n = v / nrm;

    // 计算旋转角度
    double theta = nrm;

    // 构造四元数
    double w = std::cos(theta / 2);
    double x = v_n.x() * std::sin(theta / 2);
    double y = v_n.y() * std::sin(theta / 2);
    double z = v_n.z() * std::sin(theta / 2);

    return Eigen::Quaterniond(w, x, y, z);
  }
}

}  // namespace auto_aim