#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include "tasks/auto_aim/yolo11.hpp"
#include "tasks/auto_aim/yolov8.hpp"

namespace tools
{
struct Frame
{
  int id;
  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  Eigen::Quaterniond q;
  std::list<auto_aim::Armor> armors;
};

std::vector<auto_aim::YOLO11> create_yolo11s(
  const std::string & config_path, int numebr, bool debug)
{
  std::vector<auto_aim::YOLO11> yolo11s;
  for (int i = 0; i < numebr; i++) {
    yolo11s.push_back(auto_aim::YOLO11(config_path, debug));
  }
  return yolo11s;
}

std::vector<auto_aim::YOLOV8> create_yolov8s(
  const std::string & config_path, int numebr, bool debug)
{
  std::vector<auto_aim::YOLOV8> yolov8s;
  for (int i = 0; i < numebr; i++) {
    yolov8s.push_back(auto_aim::YOLOV8(config_path, debug));
  }
  return yolov8s;
}

class ThreadPool
{
public:
  // 构造函数：创建线程池并启动指定数量的线程
  ThreadPool(size_t num_threads) : stop(false)
  {
    for (size_t i = 0; i < num_threads; ++i) {
      workers.emplace_back([this] {
        while (true) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(queue_mutex);
            condition.wait(lock, [this] { return stop || !tasks.empty(); });
            if (stop && tasks.empty()) {
              return;
            }
            task = std::move(tasks.front());
            tasks.pop();
          }
          task();
        }
      });
    }
  }

  // Destructor: Stops the thread pool and clears remaining tasks
  ~ThreadPool()
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      stop = true;
      // Clear any remaining tasks in the queue
      tasks = std::queue<std::function<void()>>();
    }
    condition.notify_all();
    for (std::thread & worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  // 添加任务到任务队列
  template <class F>
  void enqueue(F && f)
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      if (stop) {
        throw std::runtime_error("enqueue on stopped ThreadPool");
      }
      tasks.emplace(std::forward<F>(f));
      tools::logger()->debug("now tasks size: {}", tasks.size());
    }
    condition.notify_one();
  }

private:
  std::vector<std::thread> workers;         // 工作线程
  std::queue<std::function<void()>> tasks;  // 任务队列
  std::mutex queue_mutex;                   // 任务队列互斥锁
  std::condition_variable condition;        // 条件变量，用于等待任务
  bool stop;                                // 是否停止线程池
};
}  // namespace tools
