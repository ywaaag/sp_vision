#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <atomic>
#include <string>
#include <thread>

#include "serial/serial.h"

namespace io
{
struct __attribute__((packed)) GimbalFrame
{
  uint8_t head[2];
  float wxyz[4];
  float yaw;
  float vyaw;
  float pitch;
  float vpitch;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  double yaw;
  double vyaw;
  double pitch;
  double vpitch;

  void send(
    double yaw, double vyaw, double yaw_torque, double pitch, double vpitch, double pitch_torque);

private:
  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_;

  GimbalFrame data_;

  void read_thread();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP