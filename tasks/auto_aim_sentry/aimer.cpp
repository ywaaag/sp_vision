#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  if (targets.empty()) return {false, false, 0, 0};
  auto target = targets.front();

  if (bullet_speed < 10) bullet_speed = 27;

  // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
  auto future = timestamp;
  if (to_now) {
    auto dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + 0.1;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 迭代 TODO 改为循环

  future += std::chrono::microseconds(int(trajectory0.fly_time * 1e6));
  target.predict(future);

  auto aim_point1 = choose_aim_point(target);
  debug_aim_point = aim_point1;
  if (!aim_point1.valid) {
    // tools::logger()->debug("Invalid aim_point1.");
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz1 = aim_point1.xyza.head(3);
  auto d1 = std::sqrt(xyz1[0] * xyz1[0] + xyz1[1] * xyz1[1]);
  tools::Trajectory trajectory1(bullet_speed, d1, xyz1[2]);
  if (trajectory1.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d1, xyz1[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  auto time_error = trajectory1.fly_time - trajectory0.fly_time;
  if (std::abs(time_error) > 0.02) {
    tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  auto yaw = std::atan2(xyz1[1], xyz1[0]) + yaw_offset_;
  auto pitch = trajectory1.pitch + pitch_offset_;
  return {true, false, yaw, pitch};
}

AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();

  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺
  if (std::abs(ekf_x[7]) <= 2) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }

    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {false, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return {true, armor_xyza_list[lock_id_]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > comming_angle_) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle_) return {true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle_) return {true, armor_xyza_list[i]};
  }

  return {false, armor_xyza_list[0]};
}

// io::Command Aimer::aim_center(
//   std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
//   bool to_now = true)
// {
//   if (targets.empty()) return {false, false, 0, 0};
//   auto target = targets.front();

//   if (bullet_speed < 10) bullet_speed = 27;

//   // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
//   auto future = timestamp;
//   if (to_now) {
//     auto dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + 0.1;
//     future += std::chrono::microseconds(int(dt * 1e6));
//     target.predict(future);
//   }

//   auto aim_point0 = choose_aim_point(target);
//   debug_aim_point = aim_point0;
//   if (!aim_point0.valid) {
//     // tools::logger()->debug("Invalid aim_point0.");
//     return {false, false, 0, 0};
//   }

//   Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
//   auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
//   tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
//   if (trajectory0.unsolvable) {
//     tools::logger()->debug(
//       "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
//     debug_aim_point.valid = false;
//     return {false, false, 0, 0};
//   }

//   // 迭代 TODO 改为循环

//   future += std::chrono::microseconds(int(trajectory0.fly_time * 1e6));
//   target.predict(future);

//   auto aim_point1 = choose_aim_point(target);
//   debug_aim_point = aim_point1;
//   if (!aim_point1.valid) {
//     // tools::logger()->debug("Invalid aim_point1.");
//     return {false, false, 0, 0};
//   }

//   Eigen::Vector3d xyz1 = aim_point1.xyza.head(3);
//   auto d1 = std::sqrt(xyz1[0] * xyz1[0] + xyz1[1] * xyz1[1]);
//   tools::Trajectory trajectory1(bullet_speed, d1, xyz1[2]);
//   if (trajectory1.unsolvable) {
//     tools::logger()->debug(
//       "[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d1, xyz1[2]);
//     debug_aim_point.valid = false;
//     return {false, false, 0, 0};
//   }

//   auto time_error = trajectory1.fly_time - trajectory0.fly_time;
//   if (std::abs(time_error) > 0.02) {
//     tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
//     debug_aim_point.valid = false;
//     return {false, false, 0, 0};
//   }

//   auto yaw = std::atan2(xyz1[1], xyz1[0]) + yaw_offset_;
//   auto pitch = trajectory1.pitch + pitch_offset_;
//   return {true, false, yaw, pitch};
// }

}  // namespace auto_aim