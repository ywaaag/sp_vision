#ifndef TOOLS__RANSAC_SINE_FITTER_HPP
#define TOOLS__RANSAC_SINE_FITTER_HPP

#include <vector>
#include <cmath>
#include <random>
#include <Eigen/Dense>
#include <iostream>
#include <ceres/ceres.h>

#include "tools/logger.hpp"


namespace tools
{
struct SineResidual {
    SineResidual(double y, double x) : y_(y), x_(x) {}

    template <typename T>
    bool operator()(const T* const params, T* residual) const {
        T A = params[0];
        T omega = params[1];
        T phi = params[2];
        residual[0] = T(y_) - (A * ceres::sin(omega * T(x_) + phi) + T(2.090) - A);
        return true;
    }

private:
    const double y_;
    const double x_;
};

class RansacSineFitter
{
public:
    RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega, double min_A, double max_A);

    bool fit();

    bool add_data(const double y, const double x) {fit_data_.push_back(std::make_pair(y, x)); return fit_data_.size() >= 100;}

private:
    int max_iterations_;
    double threshold_;
    double min_omega_, max_omega_;
    double min_A_, max_A_;
    std::vector<std::pair<double, double>> fit_data_;
    double best_params_[3]; // 0-A 1-omega 2-phi


    bool fit_partial_model(const std::vector<double>& t, const std::vector<double>& y, double omega,
                           const std::vector<int>& indices, Eigen::Vector3d& params);

    int evaluate_model(const std::vector<double>& t, const std::vector<double>& y, double A, double omega,
                       double phi, double C, double threshold, std::vector<bool>& inlier_mask);

    double sine_function(double x, double A, double omega, double phi)
    {
        return A * std::sin(omega * x + phi) + 2.090 - A;
    }

    double compute_residual(double y, double x, double A, double omega, double phi)
    {
        return y - sine_function(x, A, omega, phi);
    }
};

} // namespace tools

#endif // TOOLS__RANSAC_SINE_FITTER_HPP