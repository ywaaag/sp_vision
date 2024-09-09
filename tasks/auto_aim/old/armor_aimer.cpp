#include "armor_aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
ArmorAimer::ArmorAimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
}

io::Command ArmorAimer::aim(
  ArmorTarget target, std::chrono::steady_clock::time_point timestamp, double bullet_speed)
{
  // 假设本函数用时可以忽略不计
  auto to_now = tools::delta_time(std::chrono::steady_clock::now(), timestamp);
  target.position += target.velocity * to_now;

  auto x0 = target.position[0];
  auto y0 = target.position[1];
  auto z0 = target.position[2];
  auto d0 = std::sqrt(x0 * x0 + y0 * y0);
  tools::Trajectory trajectory0(bullet_speed, d0, z0);
  if (trajectory0.unsolvable) {
    tools::logger()->debug("Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, z0);
    return {false, false, 0, 0};
  }

  target.position += target.velocity * trajectory0.fly_time;

  auto x1 = target.position[0];
  auto y1 = target.position[1];
  auto z1 = target.position[2];
  auto d1 = std::sqrt(x1 * x1 + y1 * y1);
  tools::Trajectory trajectory1(bullet_speed, d1, z1);
  if (trajectory1.unsolvable) {
    tools::logger()->debug("Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d1, z1);
    return {false, false, 0, 0};
  }

  auto time_error = std::abs(trajectory1.fly_time - trajectory0.fly_time);
  if (time_error > 0.01) {
    tools::logger()->debug("Large time error: {:.3f}", time_error);
    return {false, false, 0, 0};
  }

  auto yaw = std::atan2(y1, x1) + yaw_offset_;
  auto pitch = trajectory1.pitch + pitch_offset_;
  return {true, false, yaw, pitch};
}

}  // namespace auto_aim