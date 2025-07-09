#include "ransac_sine_fitter.hpp"

#include <vector>
#include <cmath>
#include <random>
#include <Eigen/Dense>
#include <iostream>
#include <ceres/ceres.h>

#include "tools/logger.hpp"

namespace tools {
    
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

bool RansacSineFitter::fit()
{
    std::vector<double> y_samples, x_sample;
    int best_inlier_count = 0;
    std::vector<bool> best_inliers;

    for (int i = max_iterations_; i > 0; --i) 
    { 
        auto samples = tools::vector_random_pick(fit_data_, 3);
        for (auto sample : samples) {
            y_samples.push_back(sample.first);
            x_sample.push_back(sample.second);
        }

        double params[3] = {1.0, 1.0, 0.0};
        ceres::Problem problem;
        for (int i = 0; i < 3; ++i) {
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<SineResidual, 1, 3>(
                    new SineResidual(y_samples[i], x_sample[i]));
            problem.AddResidualBlock(cost_function, nullptr, params);
        }

        ceres::Solver::Options options;
        options.max_num_iterations = 50; // 单次非线性拟合的迭代次数
        options.minimizer_progress_to_stdout = false;
        options.linear_solver_type = ceres::DENSE_QR;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // 3. Count inliers
        size_t inlier_count = 0;
        std::vector<bool> inliers(fit_data_.size(), false);
        for (size_t i = 0; i < inliers.size(); ++i) {
            double res = compute_residual(params[0], params[1], params[2], fit_data_[i].first, fit_data_[i].second);
            if (res < threshold_) {
                inliers[i] = true;
                ++inlier_count;
            }
        }

        // 4. Save if best
        if (inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;
            best_inliers = inliers;
            std::copy(params, params + 3, best_params_);
        }
    }

    return false;
}

// bool RansacSineFitter::fit_partial_model(const std::vector<double>& t, const std::vector<double>& y,
//                            double omega, const std::vector<int>& indices,
//                            Eigen::Vector3d& params) {

// }

// int RansacSineFitter::evaluate_model(const std::vector<double>& t, const std::vector<double>& y, double A, double omega,
//                    double phi, double C, double threshold, std::vector<bool>& inlier_mask)
// {

// }

} // namespace tools
