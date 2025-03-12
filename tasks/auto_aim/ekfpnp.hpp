#ifndef AUTO_AIM__EKFPNP_HPP
#define AUTO_AIM__EKFPNP_HPP

#include <Eigen/Dense>
#include <chrono>
#include <opencv2/core/eigen.hpp>
#include <string>

namespace auto_aim
{
class EKFPnP
{
public:
  EKFPnP(const std::string & config_path);

  void init_EKFPnP(
    const Eigen::Vector3d & c, const Eigen::Quaterniond q0,
    std::chrono::steady_clock::time_point t);

  void deinit_EKFPnP();

  void predict(std::chrono::steady_clock::time_point t);

  void update(
    const std::vector<cv::Point3f> & cvpoints_in_world,
    const std::vector<cv::Point2f> & points_in_pixel);

  void iterate_EKFPnP(
    Eigen::Vector3d & xyz_in_camera, Eigen::Matrix3d & R_armor2camera,
    std::chrono::steady_clock::time_point t, const std::vector<cv::Point3f> & cvpoints_in_world,
    const std::vector<cv::Point2f> & cvpoints_in_pixel);

private:
  Eigen::Matrix<double, 4, 3> dqomegadt_by_domega(const Eigen::Vector3d & omega, double dt);

  double dq0_by_domegaA(double omegaA, double omega, double dt);
  double dqA_by_domegaB(double omegaA, double omegaB, double omega, double dt);
  double dqA_by_domegaA(double omegaA, double omega, double dt);

  Eigen::Matrix4d dq3_by_dq1(const Eigen::Quaterniond & q);

  Eigen::Matrix4d normJac(const Eigen::Quaterniond & q);

  Eigen::MatrixXd dfh_by_ds(
    const Eigen::Matrix<double, 13, 1> & s, const Eigen::Matrix<double, 3, 4> & X,
    const Eigen::Matrix<double, 3, 4> & X_C, const Eigen::Vector4d & dist);

  Eigen::Quaterniond v2q(const Eigen::Vector3d & v);

  bool is_initialized_;

  std::chrono::steady_clock::time_point t_;

  Eigen::MatrixXd R_ = Eigen::MatrixXd::Identity(8, 8);  //观测噪声协方差矩阵 8x8
  Eigen::VectorXd X_;                                    //状态向量 13x1
  Eigen::MatrixXd P_;                                    //状态向量的协方差矩阵 13x13
  Eigen::Vector3d V0_;                                   //线速度噪声向量 3x1
  Eigen::Vector3d Omega0_;                               //角速度噪声向量 3x1
  Eigen::Vector3d linear_a_noise_;                       //线加速度噪声 3x1（可置0）
  Eigen::Vector3d angular_a_noise_;                      //角加速度噪声 3x1（可置0）
  Eigen::MatrixXd Q0_;                                   //预测过程噪声的协方差矩阵 6x6
  Eigen::Vector4d K_;                                    //相机内参
  Eigen::VectorXd Q_;
  Eigen::VectorXd P0_;
  Eigen::MatrixXd K_gain_;
};
}  // namespace auto_aim

#endif