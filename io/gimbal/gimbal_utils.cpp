#include <tuple>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include "io/gimbal/gimbal.hpp"

namespace io {

namespace {
// 计算两个时间点之间的差值（毫秒）
double delta_time(const std::chrono::steady_clock::time_point& t1, 
                 const std::chrono::steady_clock::time_point& t2) {
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t2).count();
}
}

std::tuple<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>,
          std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
Gimbal::find_adjacent_quaternions(
    const std::vector<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>& data,
    std::chrono::steady_clock::time_point t) const
{
    // 如果数据为空或只有一个元素，返回相同的值作为前后两个点
    if (data.empty()) {
        return std::make_tuple(
            std::make_tuple(Eigen::Quaterniond::Identity(), t),
            std::make_tuple(Eigen::Quaterniond::Identity(), t)
        );
    }
    
    if (data.size() == 1) {
        return std::make_tuple(data[0], data[0]);
    }

    // 二分查找找到第一个时间戳大于t的元素
    size_t left = 0;
    size_t right = data.size() - 1;
    
    // 如果目标时间在所有数据之前
    if (delta_time(t, std::get<1>(data[0])) <= 0) {
        return std::make_tuple(data[0], data[0]);
    }
    
    // 如果目标时间在所有数据之后
    if (delta_time(std::get<1>(data[right]), t) <= 0) {
        return std::make_tuple(data[right], data[right]);
    }

    while (left < right - 1) {
        size_t mid = (left + right) / 2;
        auto delta = delta_time(std::get<1>(data[mid]), t);
        
        if (delta == 0) {
            return std::make_tuple(data[mid], data[mid]);
        } else if (delta < 0) {
            left = mid;
        } else {
            right = mid;
        }
    }

    return std::make_tuple(data[left], data[right]);
}

}  // namespace io