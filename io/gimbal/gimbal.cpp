#include "gimbal.hpp"

#include "tools/logger.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path) : quit_(false)
{
  serial_.setPort("/dev/ttyACM0");
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
  double yaw, double vyaw, double yaw_torque, double pitch, double vpitch, double pitch_torque)
{
}

void Gimbal::read_thread()
{
  while (!quit_) {
    if (!serial_.isOpen()) {
      tools::logger()->warn("[Gimbal] Serial is not open!");
      continue;
    }

    serial_.read((uint8_t *)(&data_.head), sizeof(data_.head));

    if (data_.head[0] != 0xAA || data_.head[1] != 0xBB) {
      tools::logger()->warn("[Gimbal] Invalid header: {:#x} {:#x}", data_.head[0], data_.head[1]);
      continue;
    }

    serial_.read((uint8_t *)(&data_.wxyz), sizeof(GimbalFrame) - sizeof(data_.head));
    yaw = data_.yaw;
    vyaw = data_.vyaw;
    pitch = data_.pitch;
    vpitch = data_.vpitch;

    tools::logger()->debug(
      "[Gimbal] yaw: {:.2f}, vyaw: {:.2f}, pitch: {:.2f}, vpitch: {:.2f}", yaw, vyaw, pitch,
      vpitch);
  }
}

}  // namespace io