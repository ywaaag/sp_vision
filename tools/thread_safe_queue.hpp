#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>

namespace tools
{
template <typename T>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(
    size_t max_size, std::function<void(void)> full_handler = [] {})
  : max_size_(max_size), full_handler_(full_handler)
  {
  }

  // 复制构造函数
  ThreadSafeQueue(const ThreadSafeQueue & other)
  {
    std::unique_lock<std::mutex> lock(other.mutex_);
    queue_ = other.queue_;
    max_size_ = other.max_size_;
    full_handler_ = other.full_handler_;
  }

  // 赋值运算符
  ThreadSafeQueue & operator=(const ThreadSafeQueue & other)
  {
    if (this != &other) {
      std::unique_lock<std::mutex> lock1(mutex_, std::defer_lock);
      std::unique_lock<std::mutex> lock2(other.mutex_, std::defer_lock);
      std::lock(lock1, lock2);
      queue_ = other.queue_;
      max_size_ = other.max_size_;
      full_handler_ = other.full_handler_;
    }
    return *this;
  }

  void push(const T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.size() >= max_size_) {
      full_handler_();
      return;
    }

    queue_.push(value);
    not_empty_condition_.notify_all();
  }

  void pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to pop from an empty queue." << std::endl;
      return;
    }

    value = queue_.front();
    queue_.pop();
  }

  void back(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      std::cerr << "Error: Attempt to access the back of an empty queue." << std::endl;
      return;
    }

    value = queue_.back();
  }

  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
    not_empty_condition_.notify_all();  // 如果其他线程正在等待队列不为空，这样可以唤醒它们
  }

  // 获取队列的最后一个元素
  T back() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      throw std::runtime_error("Queue is empty, cannot access back element.");
    }
    return queue_.back();
  }

private:
  std::queue<T> queue_;
  size_t max_size_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_condition_;
  std::function<void(void)> full_handler_;
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP