#ifndef TOOLS__UNSCENTED_KALMAN_FILTER_HPP
#define TOOLS__UNSCENTED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <functional>

namespace tools
{
    class UnscentedKalmanFilter
    {
        public:
            // 状态维度 x, y, z, v, theta, omega
            static const int state_dim = 6;
            // 观测值维度 四个点的xyz坐标 4*3=12
            static const int observe_dim = 12;

            // UKF参数
            double alpha = 1e-3;
            double beta = 2;
            double kappa = 0;
            double lambda = alpha * alpha * (state_dim + kappa) - state_dim;

            // 状态向量
            Eigen::VectorXd x;
            // 协方差矩阵
            Eigen::MatrixXd P;
            // 时间步长
            double dt;

            UnscentedKalmanFilter(int a);
            // 构造函数
            UnscentedKalmanFilter(const Eigen::VectorXd &x_init, const Eigen::MatrixXd &P_init, double dt)
                : x(x_init), P(P_init), dt(dt) {}

            // 观测函数 计算出四个点的坐标
            Eigen::VectorXd h(const Eigen::VectorXd &x, const Eigen::MatrixXd &points_initial);
            
            // CTRV模型状态预测方程
            Eigen::VectorXd f(const Eigen::VectorXd &x, double dt);

            // sigma点生成函数
            Eigen::MatrixXd generate_sigma_points(const Eigen::VectorXd &x, const Eigen::MatrixXd &P);
            
            // 预测函数
            void predict();

            // 更新函数
            void update(const Eigen::VectorXd &z, const Eigen::MatrixXd &R, const Eigen::MatrixXd &points_initial);

    };
}

#endif