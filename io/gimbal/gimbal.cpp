#include "gimbal.hpp"
#include <cmath>
#include <cstring>
#include <fmt/core.h>

#include "io/command.hpp"
#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io {
Gimbal::Gimbal(const std::string &config_path) {
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  // 可选波特率，默认 115200（如需更高可在 YAML 中增加 baudrate 字段）
  int baudrate = 115200;
  try {
    if (yaml["baudrate"])
      baudrate = yaml["baudrate"].as<int>();
  } catch (...) {
    // ignore, keep default
  }

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baudrate);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    auto timeout = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception &e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal() {
  quit_ = true;
  if (thread_.joinable())
    thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const {
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

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t) {
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a)
      return q_c;
    if (!(t_a < t && t <= t_b))
      continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal) {
  // 兼容旧接口：映射到 32 字节定长协议
  const bool fire = (VisionToGimbal.mode == 2);
  send(/*control=*/VisionToGimbal.mode != 0, fire, VisionToGimbal.yaw,
       VisionToGimbal.yaw_vel, VisionToGimbal.yaw_acc, VisionToGimbal.pitch,
       VisionToGimbal.pitch_vel, VisionToGimbal.pitch_acc);
}

void Gimbal::send(const io::Command &cmd) {
  // 复用 io::Command，携带 horizon_distance
  // 仍然走 32 字节固定协议：把 distance 字段填入 0x0C-0x0F
  uint8_t buf[32] = {0};
  buf[0x00] = 0xFF;
  buf[0x01] = cmd.shoot ? 1 : 0;
  buf[0x02] = 0x00; // is_small_gyro，可按需接入
  buf[0x03] = 0x00; // is_navigate，可按需接入

  auto put_float = [&](size_t idx, float v) { std::memcpy(&buf[idx], &v, 4); };
  // pitch(rad)
  put_float(0x04, static_cast<float>(cmd.pitch));
  // yaw(deg)
  float yaw_deg = static_cast<float>(cmd.yaw * 180.0 / M_PI);
  put_float(0x08, yaw_deg);
  // distance(m)
  put_float(0x0C, static_cast<float>(cmd.horizon_distance));

  using namespace std::chrono;
  auto now = system_clock::now();
  auto ns = duration_cast<nanoseconds>(now.time_since_epoch()).count();
  uint32_t sec = static_cast<uint32_t>(ns / 1000000000LL);
  uint32_t nsec = static_cast<uint32_t>(ns % 1000000000LL);
  std::memcpy(&buf[0x10], &sec, 4);
  std::memcpy(&buf[0x14], &nsec, 4);

  buf[0x1F] = 0x0D;
  // fmt::print("[TX cmd] ");
  // for (auto b : buf)
  //   fmt::print("{:02X} ", b);
  // fmt::print("\n");

  try {
    serial_.write(buf, sizeof(buf));
  } catch (const std::exception &e) {
    tools::logger()->warn("[Gimbal] Failed to write serial (cmd): {}",
                          e.what());
  }
}

void Gimbal::send(bool control, bool fire, float yaw, float yaw_vel,
                  float yaw_acc, float pitch, float pitch_vel,
                  float pitch_acc) {
  // 新协议：32 字节定长包（上位机 -> 下位机）
  // 0: SOF (0xFF)
  // 1: fire_advice (0x01/0x00)
  // 2: is_small_gyro (0x01/0x00) -> 暂置 0
  // 3: is_navigate (0x01/0x00)   -> 暂置 0
  // 4-7:  pitch (float, rad)
  // 8-11: yaw   (float, deg)
  // 12-15: distance (float, m)   -> 暂置 0.0f
  // 16-19: sec (uint32)
  // 20-23: nsec (uint32)
  // 24-30: reserved (7 bytes 0x00)
  // 31: EOF (0x0D)

  uint8_t buf[32] = {0};
  buf[0x00] = 0xFF;
  buf[0x01] = fire ? 1 : 0;
  buf[0x02] = 0x00; // is_small_gyro
  buf[0x03] = 0x00; // is_navigate

  auto put_float = [&](size_t idx, float v) {
    static_assert(sizeof(float) == 4, "float must be 4 bytes");
    std::memcpy(&buf[idx], &v, 4);
  };

  put_float(0x04, pitch);
  // yaw 按协议使用角度制（deg）
  float yaw_deg = yaw * 180.0f / static_cast<float>(M_PI);
  put_float(0x08, yaw_deg);
  float distance = 0.0f;
  put_float(0x0C, distance);

  // 时间戳：使用系统时钟（秒、纳秒）
  using namespace std::chrono;
  auto now = system_clock::now();
  auto ns = duration_cast<nanoseconds>(now.time_since_epoch()).count();
  uint32_t sec = static_cast<uint32_t>(ns / 1000000000LL);
  uint32_t nsec = static_cast<uint32_t>(ns % 1000000000LL);
  std::memcpy(&buf[0x10], &sec, 4);
  std::memcpy(&buf[0x14], &nsec, 4);

  // 0x18-0x1E 7 字节保留为 0x00；当前版本不校验校验位
  buf[0x1F] = 0x0D; // EOF

  try {
    serial_.write(buf, sizeof(buf));
  } catch (const std::exception &e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t *buffer, size_t size) {
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception &e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread() {
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn(
          "[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    // 解析 32 字节固定帧（下位机 -> 上位机）
    // 帧格式：0x00 SOF(0xFF), 0x1F EOF(0x0D)
    uint8_t header = 0;
    if (!read(&header, 1)) {
      error_count++;
      continue;
    }
    if (header != 0xFF)
      continue; // 未对齐到帧头

    uint8_t buf[31] = {0};
    if (!read(buf, sizeof(buf))) { // 读取余下 31 字节
      error_count++;
      continue;
    }
    if (buf[30] != 0x0D) { // EOF 校验
      error_count++;
      continue;
    }

    auto t = std::chrono::steady_clock::now();

    // 按协议字段解析
    auto get_float = [&](size_t idx) {
      float v;
      std::memcpy(&v, &buf[idx], 4);
      return v;
    };
    float pitch = get_float(0x04 - 1); // pitch 本就为弧度
    float yaw = get_float(0x08 - 1);   // yaw 同样是弧度，取消任何角度转换

    Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = Rz * Ry;
    Eigen::Vector3d ypr = tools::eulers(q.toRotationMatrix(), 2, 1, 0);
    constexpr double kRad2Deg = 180.0 / M_PI;
    // fmt::print("[Gimbal RX] yaw: {:.2f}, pitch: {:.2f}, roll: {:.2f}
    // (deg)\n",
    //            ypr[0] * kRad2Deg, ypr[1] * kRad2Deg, ypr[2] * kRad2Deg);

    queue_.push({q, t});
    std::lock_guard<std::mutex> lock(mutex_);
    state_.yaw = yaw;
    state_.pitch = pitch;
    state_.yaw_vel = 0.0f;   // 协议未提供，置 0
    state_.pitch_vel = 0.0f; // 协议未提供，置 0
    // state_.bullet_speed 保持上次或由配置获取
    // state_.bullet_count 不在协议中，保持不变
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect() {
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...",
                          i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open(); // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception &e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

} // namespace io