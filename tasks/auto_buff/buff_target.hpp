#ifndef AUTO_BUFF__TARGET_HPP
#define AUTO_BUFF__TARGET_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>
#include <thread>
#include <ceres/ceres.h>
#include <shared_mutex>

#include "buff_detector.hpp"
#include "buff_type.hpp"
#include "tools/extended_kalman_filter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

namespace auto_buff
{
class Voter
{
public:
  Voter();
  void vote(const double angle_last, const double angle_now);
  int clockwise();

private:
  int clockwise_;
};

/// Target 基类

class Target
{
public:
  Target();
  virtual void get_target(
    const std::optional<PowerRune> & p,
    std::chrono::steady_clock::time_point & timestamp) = 0;  // 纯虚函数

  virtual void predict(double dt) = 0;  // 纯虚函数

  Eigen::Vector3d point_buff2world(const Eigen::Vector3d & point_in_buff) const;

  bool is_unsolve() const;

  Eigen::VectorXd ekf_x() const;

  double spd = 0;  //调试用

protected:
  virtual void init(double nowtime, const PowerRune & p) = 0;  // 纯虚函数

  virtual void update(double nowtime, const PowerRune & p) = 0;  // 纯虚函数

  Eigen::VectorXd x0_;
  Eigen::MatrixXd P0_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  tools::ExtendedKalmanFilter ekf_;
  double lasttime_ = 0;
  Voter voter;  // 逆时针-1 顺时针1
  bool first_in_;
  bool unsolvable_;
};

/// SmallTarget子类

class SmallTarget : public Target
{
public:
  SmallTarget();

  void get_target(
    const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp) override;

  void predict(double dt) override;

private:
  void init(double nowtime, const PowerRune & p) override;

  void update(double nowtime, const PowerRune & p) override;

  Eigen::MatrixXd h_jacobian() const;

  const double SMALL_W = CV_PI / 3;
  // const double SMALL_W = 0;
};

enum class Convexity { UNKNOWN, CONCAVE, CONVEX };                    // 拟合曲线凹凸性

/// BigTarget子类

class BigTarget : public Target
{
public:
  BigTarget(); 
  BigTarget(bool debug) : debug_(debug) {};

  // 不拷贝
  BigTarget(const BigTarget& other);                  // 拷贝构造
  BigTarget& operator=(const BigTarget& other);       // 拷贝赋值

  void get_target(
    const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp) override;

  void get_target_by_fitter(
    const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp);

  void predict(double dt) override;

  void predict_by_fitter(double dt);

private:
  void init(double nowtime, const PowerRune & p) override;

  void update(double nowtime, const PowerRune & p) override;

  Eigen::MatrixXd h_jacobian() const;

  void fit();
  bool fitOnce();

  std::array<double, 5> params_;                    // 拟合参数
  Convexity convexity_;                              // 拟合数据凹凸性
  std::shared_mutex mutex_;

  std::thread fit_thread_; // 拟合线程
  std::vector<std::pair<double, double>> fit_data_;  // 拟合数据

  const int FPS = 60;
  const int MIN_FIT_DATA_SIZE = 20;  // 最小拟合数据量
  const int MAX_FIT_DATA_SIZE = 1200;  // 最大拟合数据量

  double last_angle_ = 0;
  int total_shift_ = 0;

  bool debug_ = false;
};

/**
 * @brief 惩罚项，让拟合的参数更加贴近预设的参数
 */
class CostFunctor1 : public ceres::SizedCostFunction<1, 5> {
  public:
   CostFunctor1(double truth_, int id_) : truth(truth_), id(id_) {}
   virtual ~CostFunctor1(){};
   virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
       double pre = parameters[0][id];
       residuals[0] = pre - truth;
       if (jacobians != nullptr) {
           if (jacobians[0] != nullptr) {
               for (int i = 0; i < 5; ++i) {
                   if (i == id) {
                       jacobians[0][i] = 1;
                   } else {
                       jacobians[0][i] = 0;
                   }
               }
           }
       }
       return true;
   }
   double truth;
   int id;
};

/**
* @brief 拟合项
*/
class CostFunctor2 : public ceres::SizedCostFunction<1, 5> {
  public:
   CostFunctor2(double t_, double y_) : t(t_), y(y_) {}
   virtual ~CostFunctor2(){};
   virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
       double a = parameters[0][0];
       double w = parameters[0][1];
       double t0 = parameters[0][2];
       double b = parameters[0][3];
       double c = parameters[0][4];
       double cs = cos(w * (t + t0));
       double sn = sin(w * (t + t0));
       residuals[0] = -a * cs + b * t + c - y;
       if (jacobians != NULL) {
           if (jacobians[0] != NULL) {
               jacobians[0][0] = -cs;
               jacobians[0][1] = a * (t + t0) * sn;
               jacobians[0][2] = a * w * sn;
               jacobians[0][3] = t;
               jacobians[0][4] = 1;
           }
       }
       return true;
   }
   double t, y;
};

Convexity getConvexity(const std::vector<std::pair<double, double>> &data);
std::array<double, 5> ransacFitting(const std::vector<std::pair<double, double>> &data, Convexity convexity);
std::array<double, 5> leastSquareEstimate(const std::vector<std::pair<double, double>> &data,
                                          const std::array<double, 5> &params, Convexity convexity);


/**
 * @brief 得到大符角度，注意这里是利用参数计算出来的，相对于第一次识别的角度
 * @param[in] time          时间
 * @param[in] params        参数
 * @return double
 */
inline double getAngleBig(double time, const std::array<double, 5> &params) noexcept {
  return -params[0] * std::cos(params[1] * (time + params[2])) + params[3] * time + params[4];
};

}  // namespace auto_buff
#endif