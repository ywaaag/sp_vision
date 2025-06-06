#include "gimbal.hpp"

#include <functional>

#include "tools/logger.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
: can_("can0", std::bind(&Gimbal::callback, this, std::placeholders::_1))
{
}

void Gimbal::send(double yaw_torque, double pitch_torque)
{
  can_frame frame;
  frame.can_id = 0xfe;
  frame.can_dlc = 8;
  frame.data[0] = (int16_t)(yaw_torque * 1e3) >> 8;
  frame.data[1] = (int16_t)(yaw_torque * 1e3);
  frame.data[2] = (int16_t)(pitch_torque * 1e3) >> 8;
  frame.data[3] = (int16_t)(pitch_torque * 1e3);

  try {
    can_.write(&frame);
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void Gimbal::callback(const can_frame & frame)
{
  if (frame.can_id == 0xff) {
    auto a = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e3;
    auto b = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e3;
    auto c = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e3;
    auto d = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e3;

    this->yaw = a;
    this->vyaw = b;
    this->pitch = c;
    this->vpitch = d;
  }
}
}  // namespace io