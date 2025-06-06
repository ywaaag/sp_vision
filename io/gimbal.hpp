#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <string>

#include "io/socketcan.hpp"

namespace io
{
class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  double yaw;
  double vyaw;
  double pitch;
  double vpitch;

  void send(double yaw_torque, double pitch_torque);

private:
  SocketCAN can_;

  void callback(const can_frame & frame);
};

}  // namespace io

#endif  // IO__GIMBAL_HPP