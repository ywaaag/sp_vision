#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path) : quit_(false)
{
  serial_.setPort("/dev/ttyACM1");
  serial_.open();

  if (!serial_.isOpen()) {
    tools::logger()->warn("[Gimbal] Failed to open serial!");
    exit(0);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);
  tools::logger()->info("[Gimbal] Serial opened.");
}

Gimbal::~Gimbal()
{
  quit_ = true;

  if (thread_.joinable()) {
    thread_.join();
    tools::logger()->info("[Gimbal] thread joined.");
  }

  if (serial_.isOpen()) {
    serial_.close();
    tools::logger()->info("[Gimbal] Serial closed.");
  }
}

void Gimbal::send(
  float yaw, float vyaw, float yaw_torque, float pitch, float vpitch, float pitch_torque)
{
}

void Gimbal::read_thread()
{
  while (!quit_) {
    auto size = serial_.read((uint8_t *)(&data_.head), sizeof(data_.head));
    if (size < sizeof(data_.head)) {
      continue;
    }

    if (data_.head[0] != 0xAA || data_.head[1] != 0xBB) continue;

    size = serial_.read((uint8_t *)(&data_.wxyz), sizeof(GimbalFrame) - sizeof(data_.head));
    if (size < sizeof(GimbalFrame) - sizeof(data_.head)) {
      continue;
    }

    if (!tools::check_crc16((uint8_t *)(&data_.head), sizeof(GimbalFrame))) {
      tools::logger()->warn("[Gimbal] CRC check failed.");
      continue;
    }

    yaw = data_.yaw;
    vyaw = data_.vyaw;
    pitch = data_.pitch;
    vpitch = data_.vpitch;
  }
}

}  // namespace io