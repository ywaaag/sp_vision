#include "auto_shoot_aimer.hpp"

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
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;          // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;      // degree to rad
  shooting_angle_ = yaml["shooting_angle"].as<double>() / 57.3;  // degree to rad
  delay_gimbal_ = yaml["delay_gimbal"].as<double>();
  delay_shoot_ = yaml["delay_shoot"].as<double>();
}

io::Command Aimer::aim(
  const std::list<Target> & targets, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t_img, double bullet_speed, bool to_now)
{
  if (targets.empty()) return {false, false, 0, 0};
  auto chosen_target = targets.front();

  bool choose_last = false, choose_near = false;
  // 优先击打刚打过的目标
  if (last_target_name_ != not_armor) {
    for (auto target : targets) {
      if (target.name == last_target_name_) {
        chosen_target = target;
        choose_last = true;
        last_target_name_ = target.name;
        break;
      }
    }
  }
  // 若没有刚击打过的记录，或者上次击打的目标不在tracking状态，则选取最靠近中心的
  if (!choose_last) {
    armors.sort([](const Armor & a, const Armor & b) {
      auto img_center_norm = cv::Point2f(0.5, 0.5);
      auto distance_1 = cv::norm(a.center_norm - img_center_norm);
      auto distance_2 = cv::norm(b.center_norm - img_center_norm);
      return distance_1 < distance_2;
    });

    for (auto armor : armors) {
      for (auto target : targets) {
        if (target.name == armor.name) {
          chosen_target = target;
          choose_near = true;
          last_target_name_ = target.name;
          break;
        }
      }
      if (choose_near) break;
    }
  }

  if (!choose_last && !choose_near) {
    tools::logger()->warn("ERROR: targets not empty, but refused to aim!");
    return {false, false, 0, 0};
  }

  bool can_fire = true;

  if (bullet_speed < 10) bullet_speed = 23;

  if (to_now) {
    chosen_target.predict(std::chrono::steady_clock::now());
  }

  auto aim_point0 = choose_coming_aim_point(chosen_target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    tools::logger()->debug("Invalid aim_point0.");
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

  if (to_now) {
    auto t_hit =
      std::chrono::steady_clock::now() +
      std::chrono::microseconds(int((delay_gimbal_ + delay_shoot_ + trajectory0.fly_time) * 1e6));
    chosen_target.predict(t_hit);
  } else {
    auto t_hit = t_img + std::chrono::microseconds(
                           int((delay_gimbal_ + delay_shoot_ + trajectory0.fly_time) * 1e6));
    chosen_target.predict(t_hit);
  }
  auto aim_point1 = choose_aim_point(chosen_target);
  debug_aim_point = aim_point1;
  if (!aim_point1.valid) {
    aim_point1 = choose_coming_aim_point(chosen_target);
    tools::logger()->debug("aim at coming point");
    can_fire = false;
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
  if (std::abs(time_error) > 0.1) {
    tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  auto yaw = std::atan2(xyz1[1], xyz1[0]) + yaw_offset_;
  auto pitch = trajectory1.pitch + pitch_offset_;
  return {true, can_fire, yaw, pitch};
}

void Aimer::clear_last() { last_target_name_ = not_armor; }

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
  if (std::abs(ekf_x[7]) <= 1) {
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
    if (std::abs(delta_angle_list[i]) > shooting_angle_) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < shooting_angle_) return {true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -shooting_angle_) return {true, armor_xyza_list[i]};
  }
  return {false, armor_xyza_list[0]};
}

AimPoint Aimer::choose_coming_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  Eigen::Vector3d center = {ekf_x[0], ekf_x[2], ekf_x[4]};
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  // delta_angle 为正，在车中心右侧
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  if (ekf_x[7] > 0) {  // 向右旋转
    int chosen_id = 0;
    double min_val = -1e3;
    for (int id = 0; id < armor_num; id++) {
      if (
        delta_angle_list[id] < -shooting_angle_ &&
        delta_angle_list[id] > min_val)  // 选择左侧离击打范围最近的装甲板
      {
        chosen_id = id;
        min_val = delta_angle_list[chosen_id];
      }
    }
    auto use_l_h = (armor_num == 4) && (chosen_id == 1 || chosen_id == 3);
    auto r = (use_l_h) ? ekf_x[8] + ekf_x[9] : ekf_x[8];
    return {
      true,
      {center[0] - std::cos(-shooting_angle_ + center_yaw) * r,
       center[1] - std::sin(-shooting_angle_ + center_yaw) * r, armor_xyza_list[chosen_id][2],
       -shooting_angle_ + center_yaw}};
  } else {
    int chosen_id = 0;
    double max_val = 1e3;
    for (int id = 0; id < armor_num; id++) {
      if (
        delta_angle_list[id] > shooting_angle_ &&
        delta_angle_list[id] < max_val)  // 选择右侧离击打范围最近的装甲板
      {
        chosen_id = id;
        max_val = delta_angle_list[chosen_id];
      }
    }
    auto use_l_h = (armor_num == 4) && (chosen_id == 1 || chosen_id == 3);
    auto r = (use_l_h) ? ekf_x[8] + ekf_x[9] : ekf_x[8];
    return {
      true,
      {center[0] - std::cos(shooting_angle_ + center_yaw) * r,
       center[1] - std::sin(shooting_angle_ + center_yaw) * r, armor_xyza_list[chosen_id][2],
       shooting_angle_ + center_yaw}};
  }
  return {0, armor_xyza_list[0]};  // 不会运行到这里
}

}  // namespace auto_aim