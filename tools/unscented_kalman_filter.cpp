#include <unscented_kalman_filter.hpp>

namespace tools
{
    UnscentedKalmanFilter::UnscentedKalmanFilter(int a);

    UnscentedKalmanFilter::UnscentedKalmanFilter(const Eigen::VectorXd &x_init, const Eigen::MatrixXd &P_init, double dt)
                : x(x_init), P(P_init), dt(dt) {}

    Eigen::VectorXd UnscentedKalmanFilter::h(const Eigen::VectorXd &x, const Eigen::MatrixXd &points_initial) {
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << cos(x(4)), -sin(x(4)), 0,
                            sin(x(4)), cos(x(4)), 0,
                            0, 0, 1;
        
        Eigen::MatrixXd points = points_initial * rotation_matrix.transpose();
        return Eigen::Map<Eigen::VectorXd>(points.data(), points.size());
    }

    Eigen::VectorXd UnscentedKalmanFilter::f(const Eigen::VectorXd &x, double dt) {
        Eigen::VectorXd x_new = x;
        x_new(0) += x(3) * cos(x(4)) * dt;  // x position
        x_new(1) += x(3) * sin(x(4)) * dt;  // y position
        x_new(2) = x(2);  // z position remains the same
        x_new(3) = x(3);  // velocity
        x_new(4) += x(5) * dt;  // theta (heading)
        x_new(5) = x(5);  // omega (angular velocity)
        return x_new;
    }

    // 生成Sigma点
    Eigen::MatrixXd UnscentedKalmanFilter::generate_sigma_points(const Eigen::VectorXd &x, const Eigen::MatrixXd &P) {
        Eigen::MatrixXd sigma_points(state_dim, 2 * state_dim + 1);
        Eigen::MatrixXd S = P.llt().matrixL();  // 计算协方差矩阵的Cholesky分解

        sigma_points.col(0) = x;
        for (int i = 0; i < state_dim; ++i) {
            sigma_points.col(i + 1) = x + sqrt(state_dim + lambda) * S.col(i);
            sigma_points.col(i + state_dim + 1) = x - sqrt(state_dim + lambda) * S.col(i);
        }
        return sigma_points;
    }

    void UnscentedKalmanFilter::predict() {
        // 生成Sigma点
        Eigen::MatrixXd sigma_points = generate_sigma_points(x, P);
        
        // 传播Sigma点
        Eigen::MatrixXd sigma_points_pred(state_dim, 2 * state_dim + 1);
        for (int i = 0; i < 2 * state_dim + 1; ++i) {
            sigma_points_pred.col(i) = f(sigma_points.col(i), dt);
        }

        // 计算预测的状态均值
        x = sigma_points_pred.rowwise().mean();

        // 计算预测的协方差矩阵
        P.setZero();
        for (int i = 0; i < 2 * state_dim + 1; ++i) {
            Eigen::VectorXd diff = sigma_points_pred.col(i) - x;
            P += diff * diff.transpose();
        }
        P /= (2 * state_dim + 1);
    }

    // UKF更新步骤
    void UnscentedKalmanFilter::update(const Eigen::VectorXd &z, const Eigen::MatrixXd &R, const Eigen::MatrixXd &points_initial) {
        // 生成Sigma点
        Eigen::MatrixXd sigma_points_pred = generate_sigma_points(x, P);
        
        // 计算观测值
        Eigen::MatrixXd z_pred(observe_dim, 2 * state_dim + 1);
        for (int i = 0; i < 2 * state_dim + 1; ++i) {
            z_pred.col(i) = h(sigma_points_pred.col(i), points_initial);
        }

        // 计算观测值的均值
        Eigen::VectorXd z_pred_mean = z_pred.rowwise().mean();

        // 计算创新
        Eigen::VectorXd dz = z - z_pred_mean;

        // 计算Kalman增益
        Eigen::MatrixXd P_xz(state_dim, observe_dim);
        Eigen::MatrixXd P_zz(observe_dim, observe_dim);
        P_xz.setZero();
        P_zz.setZero();
        for (int i = 0; i < 2 * state_dim + 1; ++i) {
            Eigen::VectorXd dx = sigma_points_pred.col(i) - x;
            Eigen::VectorXd dz_i = z_pred.col(i) - z_pred_mean;
            P_xz += dx * dz_i.transpose();
            P_zz += dz_i * dz_i.transpose();
        }
        P_xz /= (2 * state_dim + 1);
        P_zz /= (2 * state_dim + 1);
        P_zz += R;

        // 计算Kalman增益
        Eigen::MatrixXd K = P_xz * P_zz.inverse();

        // 更新状态和协方差
        x += K * dz;
        P -= K * P_zz * K.transpose();
    }

} // namespace tools
