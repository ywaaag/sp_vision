#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#include <fstream>
#include <iomanip>
#include <ctime>
#include <string>

// small helper to dump inbound raw bytes for debugging
static void dump_inbound_raw(const uint8_t *data, size_t len)
{
  try {
    system("mkdir -p logs");
    std::ofstream ofs("logs/gimbal_in_raw.log", std::ios::app);
    if (!ofs.is_open()) return;
    ofs << "[" << std::to_string(time(nullptr)) << "] ";
    for (size_t i = 0; i < len; ++i) {
      ofs << std::hex << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
    }
    ofs << std::dec << std::endl;
    ofs.close();
  } catch (...) {
  }
}

// minimal DM_IMU-like parser: if buffer contains 0xFF 0x01 0x00 0x00 + 3 floats + 0x0D, extract r,p,y
static bool try_parse_short_dm_imu(const uint8_t *buf, size_t len, float &roll, float &pitch, float &yaw)
{
  if (len < 4 + 12 + 1) return false;
  if (!(buf[0] == 0xFF && buf[1] == 0x01 && buf[2] == 0x00 && buf[3] == 0x00)) return false;
  // find terminator 0x0D
  size_t term_pos = SIZE_MAX;
  for (size_t i = 4 + 12; i < len; ++i) {
    if (buf[i] == 0x0D) { term_pos = i; break; }
  }
  if (term_pos == SIZE_MAX) return false;
  // enough data
  memcpy(&roll, buf + 4, sizeof(float));
  memcpy(&pitch, buf + 8, sizeof(float));
  memcpy(&yaw, buf + 12, sizeof(float));
  return true;
}

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    // configure serial parameters similar to DM_IMU
    int baud = 115200;
    if (yaml["baudrate"]) baud = yaml["baudrate"].as<int>();
    try {
      serial_.setBaudrate(static_cast<uint32_t>(baud));
    } catch (...) {
      // some serial implementations may throw for unsupported baud; ignore to allow open
    }
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(200);
    serial_.setTimeout(time_out);
    serial_.open();
    usleep(1000000); // 1s delay to allow device to settle
    tools::logger()->info("[Gimbal] serial port opened: {}", com_port);
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  // queue_.pop();
  // tools::logger()->info("[Gimbal] First q received.");
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
  auto start_time = std::chrono::steady_clock::now();
  // 首先检查缓存的最新数据
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_data_.valid) {
      double delta = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
        latest_data_.timestamp - t).count();
      if (std::abs(delta) < 0.001) { // 1ms内的时间差，直接返回缓存值
        return latest_data_.quaternion;
      }
    }
  }

  // 获取所有当前数据
  auto data = buffer_.dump();
  if (data.empty()) {
    tools::logger()->warn("[Gimbal] No quaternion data available");
    return Eigen::Quaterniond::Identity();
  }

  // 使用二分查找找到相邻的两个四元数
  auto [pair_a, pair_b] = find_adjacent_quaternions(data, t);
  auto [q_a, t_a] = pair_a;
  auto [q_b, t_b] = pair_b;

  // 计算插值参数
  double t_ab = std::chrono::duration_cast<std::chrono::duration<double>>(t_b - t_a).count();
  double t_at = std::chrono::duration_cast<std::chrono::duration<double>>(t - t_a).count();
  double k = t_ab == 0 ? 0 : t_at / t_ab;

  // 使用球面线性插值(slerp)进行四元数插值
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  // 输出性能统计
  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  static int call_count = 0;
  static double total_duration = 0;
  call_count++;
  total_duration += duration;
  
  if (call_count % 100 == 0) {
    tools::logger()->info("[Gimbal] q(t) average duration: {:.2f}us, calls: {}", 
                               total_duration / call_count, call_count);
  }

  // 更新缓存
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_data_.quaternion = q_c;
    latest_data_.timestamp = t;
    latest_data_.valid = true;
  }

  return q_c;
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

// Helper: write 32-byte FixedPacket to serial (head 0xFF, tail 0x0D)
static void write_fixed_packet(serial::Serial & serial, const uint8_t buf[32])
{
  try {
    // log hex to file for verification
    system("mkdir -p logs");
    std::ofstream ofs("logs/gimbal_out.log", std::ios::app);
    if (ofs.is_open()) {
      ofs << "[" << std::to_string(time(nullptr)) << "] ";
      for (int i = 0; i < 32; ++i) {
        ofs << std::hex << std::setw(2) << std::setfill('0') << (int)buf[i] << " ";
      }
      ofs << std::dec << std::endl;
      ofs.close();
    }
  } catch (...) {
  }

  try {
    serial.write(const_cast<uint8_t *>(buf), 32);
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write fixed packet: {}", e.what());
  }
}

void Gimbal::send_gimbal_cmd(
  uint8_t fire_advice, float pitch, float yaw, float distance, uint32_t sec, uint32_t nanosec) const
{
  uint8_t buf[32] = {0};
  buf[0] = 0xFF;  // head
  // data indices are 1-based after head; map to 0-based buf
  buf[1] = fire_advice;  // byte index 1

  // pitch: bytes 4-7 -> buf[4]
  memcpy(&buf[4], &pitch, sizeof(float));
  // yaw: bytes 8-11 -> buf[8]
  memcpy(&buf[8], &yaw, sizeof(float));
  // distance: bytes 12-15 -> buf[12]
  memcpy(&buf[12], &distance, sizeof(float));
  // timestamp sec: bytes 16-19 -> buf[16]
  memcpy(&buf[16], &sec, sizeof(uint32_t));
  // timestamp nanosec: bytes 20-23 -> buf[20]
  memcpy(&buf[20], &nanosec, sizeof(uint32_t));

  buf[31] = 0x0D;  // tail at position 31 (32nd byte)

  // write
  write_fixed_packet(const_cast<serial::Serial &>(serial_), buf);
}

void Gimbal::send_chassis_cmd(
  uint8_t is_spining, uint8_t is_navigating, float lin_x, float lin_y, float ang_z) const
{
  uint8_t buf[32] = {0};
  buf[0] = 0xFF;
  // according to your spec, byte index 2 and 3 (1-based after head) correspond to buf[2], buf[3]
  buf[2] = is_spining;
  buf[3] = is_navigating;

  // twist.linear.x: bytes 16-19 -> buf[16]
  memcpy(&buf[16], &lin_x, sizeof(float));
  // twist.linear.y: bytes 20-23 -> buf[20]
  memcpy(&buf[20], &lin_y, sizeof(float));
  // twist.angular.z: bytes 24-27 -> buf[24]
  memcpy(&buf[24], &ang_z, sizeof(float));

  buf[31] = 0x0D;

  write_fixed_packet(const_cast<serial::Serial &>(serial_), buf);
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    size_t offset = 0;
    auto start = std::chrono::steady_clock::now();
    while (offset < size) {
      size_t n = serial_.read(buffer + offset, size - offset);
      if (n > 0) offset += n;
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 500) break;
    }
    return offset == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;
  const int max_error_count = 100; // lower threshold for faster reconnect

  while (!quit_) {
    if (error_count > max_error_count) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    // try read a single byte peek to capture any inbound raw data for optional dumping
    uint8_t peek_buf[256];
    bool got_peek = false;
    if (read(peek_buf, 1)) {
      got_peek = true;
      // Try to read a small batch of bytes into the peek buffer; some serial
      // backends don't implement available() and may throw — avoid calling it.
      try {
        size_t r = serial_.read(peek_buf + 1, sizeof(peek_buf) - 1);
        (void)r; // we don't rely on exact count here
      } catch (...) {
        // ignore and continue; peek_buf contains at least the first byte
      }
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    // if we peeked and dumping enabled, write the raw bytes
    const char *env_dump = std::getenv("GIMBAL_DUMP_INBOUND");
    if (got_peek && env_dump && std::string(env_dump) == "1") {
      dump_inbound_raw(peek_buf, 1);
    }

    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') {
      // not SP frame: maybe this is a DM_IMU short frame starting with 0xFF
      // try to read a small window and parse
      uint8_t window[64] = {0};
      // we already consumed nothing relevant; try to read up to 64 bytes non-blocking
      size_t r = 0;
      try {
        r = serial_.read(window, sizeof(window));
      } catch (...) { r = 0; }
      if (r > 0) {
        // optionally dump inbound
        if (env_dump && std::string(env_dump) == "1") dump_inbound_raw(window, r);
        float roll = 0, pitch = 0, yaw = 0;
        if (try_parse_short_dm_imu(window, r, roll, pitch, yaw)) {
          // mapping same as serial_to_vcan: orig=(roll,pitch,yaw) -> mapped: r=orig_y, p=orig_r, y=-orig_p
          float m_roll = yaw;
          float m_pitch = roll;
          float m_yaw = -pitch;
          Eigen::Quaterniond q = Eigen::AngleAxisd(m_yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(m_pitch * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(m_roll * M_PI / 180.0, Eigen::Vector3d::UnitX());
          q.normalize();
          auto current_time = std::chrono::steady_clock::now();
          buffer_.push({q, current_time});
          
          // 更新最新数据缓存
          {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_data_.quaternion = q;
            latest_data_.timestamp = current_time;
            latest_data_.valid = true;
          }
          
          error_count = 0;
          continue;
        }
      }

      continue;
    }

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    
    // 更新环形缓冲区
    buffer_.push({q, t});
    
    // 更新最新数据缓存
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_data_.quaternion = q;
      latest_data_.timestamp = t;
      latest_data_.valid = true;
    }
     buffer_.push(std::make_tuple(q, t));    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;

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

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      buffer_.clear();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_data_.valid = false;
      }
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io