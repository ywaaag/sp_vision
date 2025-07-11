#include "ransac_sine_fitter.hpp"

#include <vector>
#include <cmath>
#include <random>
#include <Eigen/Dense>
#include <iostream>
#include <Eigen/Core>
#include <unsupported/Eigen/NonLinearOptimization>

#include "tools/logger.hpp"

namespace tools {
struct LMFunctor {
    const std::vector<double>& t_data;
    const std::vector<double>& v_data;
    int m; // 数据点数量
    
    LMFunctor(const std::vector<double>& t, const std::vector<double>& v)
        : t_data(t), v_data(v), m(t.size()) {}
    
    int operator()(const Eigen::VectorXd &p, Eigen::VectorXd &fvec) const {
        double A = p[0];
        double omega = p[1];
        double phi = p[2];
        double C = 2.090 - A;  // 约束条件
        
        for (int i = 0; i < m; ++i) {
            double t = t_data[i];
            double pred = A * std::sin(omega * t + phi) + C;
            fvec[i] = v_data[i] - pred;
        }
        return 0;
    }

    int df(const Eigen::VectorXd &p, Eigen::MatrixXd &fjac) const {
        double A = p[0];
        double omega = p[1];
        double phi = p[2];
        
        for (int i = 0; i < m; ++i) {
            double t = t_data[i];
            double theta = omega * t + phi;
            
            // 对A的偏导: -sin(ωt + φ) + 1 (因为C = 2.090 - A)
            fjac(i, 0) = -std::sin(theta) - 1.0;
            
            // 对ω的偏导: -A * t * cos(ωt + φ)
            fjac(i, 1) = -A * t * std::cos(theta);
            
            // 对φ的偏导: -A * cos(ωt + φ)
            fjac(i, 2) = -A * std::cos(theta);
        }
        return 0;
    }
    
    int inputs() const { return 3; } // 参数数量 (A, omega, phi)
    int values() const { return m; } // 数据点数量
};
    
template<typename T>
std::vector<T> vector_random_pick(const std::vector<T>& input, const int num_elements) {
    if (input.size() < num_elements) tools::logger()->warn("[WARN] Input vector size is less than num_elements");

    std::vector<size_t> indices(input.size());
    std::iota(indices.begin(), indices.end(), 0);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(indices.begin(), indices.end(), gen);

    std::vector<T> result;
    for (int i = 0; i < num_elements; ++i) {
        result.push_back(input[indices[i]]);
    }

    return result;
}


RansacSineFitter::RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega,
                                    double min_A, double max_A)
        : max_iterations_(max_iterations),
          threshold_(threshold),
          min_omega_(min_omega),
          max_omega_(max_omega),
          min_A_(min_A),
          max_A_(max_A) {}


bool RansacSineFitter::fit() {
    if (fit_data_.size() < 3) return false;

    int best_inliers = -1;
    std::vector<bool> best_inlier_mask;
    
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // 随机选择3个点
        auto indices = vector_random_pick(fit_data_, 3);
        if (indices.empty()) continue;
        
        // 随机选择omega
        std::uniform_real_distribution<double> omega_dist(min_omega_, max_omega_);
        double omega = omega_dist(gen_);
        
        Eigen::Vector3d coeffs;
        if (fit_linear_model(indices, omega, coeffs)) {
            // 计算非线性参数
            double tmp0 = coeffs[0];
            double tmp1 = coeffs[1];
            double tmp2 = coeffs[2];

            double A = std::sqrt(tmp0*tmp0 + tmp1*tmp1);
            double phi = std::atan2(tmp1, tmp0);
            double omega = omega;
            double C = 2.090 - A;  // 应用约束条件

            // if( A < min_A_ || A > max_A_ || omega < min_omega_ || omega > max_omega_) {
            //     continue; // 跳过不满足约束条件的模型
            // }
            
            // 获取内点及数量
            auto inlier_mask = evaluate_inliers(A, omega, phi, C);
            int inlier_count = count_inliers(inlier_mask);
            
            // 更新最佳模型
            if (inlier_count > best_inliers) {
                best_inliers = inlier_count;
                best_params_[0] = A;
                best_params_[1] = omega;
                best_params_[2] = phi;
                best_inlier_mask = inlier_mask;
            }
        }
    }
    
    if (best_inliers > 0) {
        refine_with_inliers(best_inlier_mask);
        return true;
    }
    
    return false;
}

bool RansacSineFitter::fit_linear_model(const std::vector<std::pair<double, double>>& indices, double omega, Eigen::Vector3d& coeffs) {
    Eigen::MatrixXd A(indices.size(), 3);
    Eigen::VectorXd b(indices.size());
    
    for (int i = 0; i < indices.size(); ++i) {
        double t = indices[i].second;
        double sin_val = std::sin(omega * t);
        double cos_val = std::cos(omega * t);
        
        A(i, 0) = sin_val;
        A(i, 1) = cos_val;
        A(i, 2) = 1.0;
        b(i) = indices[i].first;
    }
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (svd.rank() < 3) return false;
    
    coeffs = svd.solve(b);
    return true;
}

std::vector<bool> RansacSineFitter::evaluate_inliers(const double A, const double omega, const double phi, const double C) {
    std::vector<bool> mask(fit_data_.size(), false);

    for (int i = 0; i < fit_data_.size(); ++i) {
        double t = fit_data_[i].second;
        double prediction = sine_function(t, A, omega, phi);
        double error = std::abs(fit_data_[i].first - prediction);
        mask[i] = (error < threshold_);
    }
    
    return mask;
}

int RansacSineFitter::count_inliers(const std::vector<bool>& mask) {
    return std::count(mask.begin(), mask.end(), true);
}

void RansacSineFitter::refine_with_inliers(const std::vector<bool>& inlier_mask) {
    // 内点数据
    std::vector<double> t_inliers;
    std::vector<double> v_inliers;

    for (int i = 0; i < fit_data_.size(); ++i) {
        if (inlier_mask[i]) {
            t_inliers.push_back(fit_data_[i].second);
            v_inliers.push_back(fit_data_[i].first);
        }
    }
    
    if (t_inliers.size() < 3) return;
    
    // 定义LM优化器
    Eigen::VectorXd p(3);
    p << best_params_[0], best_params_[1], best_params_[2];

    LMFunctor functor(t_inliers, v_inliers);
    Eigen::LevenbergMarquardt<LMFunctor> lm(functor);
    lm.minimize(p);
    
    // 更新参数
    best_params_[0] = p[0];
    best_params_[1] = p[1];
    best_params_[2] = p[2];
}


} // namespace tools
