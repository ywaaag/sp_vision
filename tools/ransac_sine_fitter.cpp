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
    if (fit_data_.size() < 60) return false;

    int best_inliers = -1;
    std::vector<bool> best_inlier_mask;
    
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // 随机选择3个点
        auto indices = vector_random_pick(fit_data_, 3);
        if (indices.empty()) continue;
        
        // 随机选择omega
        std::uniform_real_distribution<double> omega_dist(min_omega_, max_omega_);
        double omega = omega_dist(gen_);

        Eigen::Vector3d params;
        if (!fit_partial_model(indices, omega, params)) {
            continue;
        }

        // 4. Calculate A and phi
        double A1 = params(0), A2 = params(1);
        double A = std::sqrt(A1 * A1 + A2 * A2);
        double phi = std::atan2(A2, A1);
        double C = params(2);

        C = 2.090 - A;

        if (A < min_A_ || A > max_A_) {
            continue;
        }

        auto inliners = evaluate_inliers(A, omega, phi, C);
        if(inliners.size() > best_inliers) {
            best_inliers = inliners.size();
            best_inlier_mask = inliners;
            best_params_[0] = A;
            best_params_[1] = omega;
            best_params_[2] = phi;
        }

    }
        
    fit_data_.clear();
    return best_inliers > 0;
}

bool RansacSineFitter::fit_partial_model(const std::vector<std::pair<double, double>> & indices, double omega, Eigen::Vector3d& params) {
        Eigen::MatrixXd X(indices.size(), 3);
        Eigen::VectorXd Y(indices.size());

        for (size_t i = 0; i < indices.size(); ++i) {
            X(i, 0) = std::sin(omega * indices[i].second);
            X(i, 1) = std::cos(omega * indices[i].second);
            X(i, 2) = 1.0;
            Y(i) = indices[i].first;
        }
        try {
            params = X.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
            return true;
        } catch (...) {
            return false;
        }
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

} // namespace tools
