#pragma once

#include <array>
#include <mutex>
#include <vector>

namespace io {

/**
 * @brief 线程安全的固定大小环形缓冲区
 */
template<typename T, std::size_t Size = 20>
class RingBuffer {
public:
    RingBuffer() : head_(0), tail_(0), count_(0) {}

    /**
     * @brief 线程安全地添加一个元素。如果缓冲区已满，最旧的元素会被覆盖。
     */
    void push(const T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_[head_] = item;
        
        if (count_ == Size) {
            // 缓冲区已满，覆盖最旧元素。tail_需要向前移动
            tail_ = (tail_ + 1) % Size;
        } else {
            // 缓冲区未满，增加计数
            ++count_;
        }
        // head_ 总是指向下一个写入位置
        head_ = (head_ + 1) % Size;
    }

    /**
     * @brief 线程安全地清空缓冲区。
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

    /**
     * @brief 线程安全地将所有元素按顺序复制到一个 std::vector 中。
     * @return 包含所有元素的 vector。
     */
    std::vector<T> dump() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<T> result;
        if (count_ == 0) {
            return result;
        }

        result.reserve(count_);
        std::size_t current = tail_; // 从最早的数据位置开始
        for (std::size_t i = 0; i < count_; ++i) {
            result.push_back(buffer_[current]);
            current = (current + 1) % Size;
        }
        return result;
    }

    /**
     * @brief 检查缓冲区是否为空。
     */
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_ == 0;
    }

    /**
     * @brief 获取当前存储的元素数量。
     */
    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_;
    }

private:
    mutable std::mutex mutex_;
    std::array<T, Size> buffer_;
    std::size_t head_;    // 写入位置
    std::size_t tail_;    // 最早数据位置
    std::size_t count_;   // 当前元素数量
};

}  // namespace io