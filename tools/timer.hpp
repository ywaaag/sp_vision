// file: tools/timer.hpp
#pragma once

#include <chrono>
#include <string>
#include "tools/logger.hpp" 
namespace tools {

class ScopedTimer {
public:
    // 构造函数：记录起始时间并保存名称
    ScopedTimer(const std::string& name)
        : name_(name), start_time_(std::chrono::steady_clock::now()) {}

    // 析构函数：在对象生命周期结束时自动调用
    ~ScopedTimer() {
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(end_time - start_time_).count();
        // 使用你的 logger 打印耗时
        tools::logger()->debug("[Timer] {}: {:.2f} ms", name_, duration);
    }

private:
    std::string name_;
    std::chrono::steady_clock::time_point start_time_;
};

} // namespace tools