#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  tx_data_.yaw_kp = tools::read<float>(yaml, "yaw_kp");
  tx_data_.yaw_kd = tools::read<float>(yaml, "yaw_kd");
  tx_data_.pitch_kp = tools::read<float>(yaml, "pitch_kp");
  tx_data_.pitch_kd = tools::read<float>(yaml, "pitch_kd");

  try {
    serial_.setPort(com_port);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

void Gimbal::send(
  bool control, float yaw, float vyaw, float yaw_torque, float pitch, float vpitch,
  float pitch_torque)
{
  tx_data_.control = control ? 1 : 0;
  tx_data_.yaw = yaw;
  tx_data_.vyaw = vyaw;
  tx_data_.yaw_torque = yaw_torque;
  tx_data_.pitch = pitch;
  tx_data_.vpitch = vpitch;
  tx_data_.pitch_torque = pitch_torque;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");

  while (!quit_) {
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_.head), sizeof(rx_data_.head))) continue;
    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_.mode), sizeof(rx_data_) - sizeof(rx_data_.head)))
      continue;

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.vyaw = rx_data_.vyaw;
    state_.pitch = rx_data_.pitch;
    state_.vpitch = rx_data_.vpitch;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Unknown mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

}  // namespace io