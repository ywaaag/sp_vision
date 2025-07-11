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
class RansacSineFitter
{
public:
    RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega, double min_A, double max_A);

    bool fit();

    void add_data(const double y, const double x) {fit_data_.push_back(std::make_pair(y, x));}

    bool fit_data_enough() {return fit_data_.size() >= 60;}

    double* get_params() { return best_params_; }
    
    double sine_function(double x, double A, double omega, double phi)
    {
        return A * std::sin(omega * x + phi) + 2.090 - A;
    }

private:
    const int max_iterations_;
    const double threshold_;
    const double min_omega_, max_omega_;
    const double min_A_, max_A_;

    std::vector<std::pair<double, double>> fit_data_;
    double best_params_[3]; // 0-A 1-omega 2-phi
    std::mt19937 gen_;

    bool fit_partial_model(const std::vector<std::pair<double, double>>& indices, double omega, Eigen::Vector3d& params);

    std::vector<bool> evaluate_inliers(const double A, const double omega, const double phi, const double C);
};

} // namespace tools

#endif // TOOLS__RANSAC_SINE_FITTER_HPP