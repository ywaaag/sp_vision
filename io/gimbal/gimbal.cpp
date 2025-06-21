#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
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

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
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

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    if (!(t_a < t && t <= t_b)) continue;

    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    return q_c;
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float vyaw, float yaw_torque, float pitch, float vpitch,
  float pitch_torque)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
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
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) continue;
    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head)))
      continue;

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.vyaw = rx_data_.vyaw;
    state_.pitch = rx_data_.pitch;
    state_.vpitch = rx_data_.vpitch;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;
    state_.spin_speed = rx_data_.spin_speed;

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
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

}  // namespace io