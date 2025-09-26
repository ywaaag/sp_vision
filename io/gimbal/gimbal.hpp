#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "io/gimbal/ring_buffer.hpp"

namespace io
{
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) <= 64);

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
};

static_assert(sizeof(VisionToGimbal) <= 64);

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(io::VisionToGimbal VisionToGimbal);
  // New: send FixedPacket-format GimbalCmd and ChassisCmd (32 bytes, head 0xFF, tail 0x0D)
  void send_gimbal_cmd(uint8_t fire_advice, float pitch, float yaw, float distance,
                       uint32_t sec, uint32_t nanosec) const;
  void send_chassis_cmd(uint8_t is_spining, uint8_t is_navigating, float lin_x, float lin_y,
                        float ang_z) const;

private:
  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  
  // 使用新的RingBuffer替换ThreadSafeQueue
  RingBuffer<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>, 20> buffer_;
  
  // 缓存最新的四元数和时间戳
  mutable struct {
    Eigen::Quaterniond quaternion;
    std::chrono::steady_clock::time_point timestamp;
    bool valid = false;
  } latest_data_;

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();
  
  // 新增：用于在时间序列中进行二分查找
  std::tuple<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>,
            std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>> 
  find_adjacent_quaternions(
    const std::vector<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>& data,
    std::chrono::steady_clock::time_point t) const;
};

}  // namespace io

#endif  // IO__GIMBAL_HPP