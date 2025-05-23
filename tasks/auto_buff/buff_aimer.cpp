#include "buff_aimer.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_buff
{
Aimer::Aimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
}

io::Command Aimer::aim(
  auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp,
  double bullet_speed, bool to_now)
{
  /* 
  状态轴：SEND_ANGLE---SEND_FIRE---WAIT
      SEND_ANGLE: 发送角度指令
      SEND_FIRE: 发送射击指令
      WAIT: 等待下一次发送角度指令
  
  label_timestamp: 每轮状态循环开始的时间戳
  */

  static std::chrono::steady_clock::time_point label_timestamp = std::chrono::steady_clock::now();
  static double yaw, pitch;

  auto now = std::chrono::steady_clock::now();
  if (target.is_unsolve()) {
    label_timestamp = now;
    reset_status_();
    return {false, false, 0, 0};
  }
  if (bullet_speed < 10) bullet_speed = 22;

  io::Command command = {false, false, 0, 0};

  auto detect_now_gap = tools::delta_time(now, timestamp);
  if (get_send_angle(target, detect_now_gap, bullet_speed, to_now, yaw, pitch)) {
    command.control = true;
    command.yaw = yaw;
    command.pitch = pitch;
  } else {
    label_timestamp = now;
    reset_status_();
    return command;
  }

  // send_angle -> send_fire
  if (status_ == SEND_ANGLE && tools::delta_time(now, label_timestamp) > AIM_TIME) {
    label_timestamp = now;
    update_status_();
  }
  // send_fire -> wait
  else if (status_ == SEND_FIRE) {
    label_timestamp = now;
    command.shoot = true;
    update_status_();
  }
  // wait -> send_fire
  else if (
    status_ == WAIT && tools::delta_time(now, label_timestamp) > WAIT_TIME) {
    update_status_();
  }

  return command;
}

bool Aimer::get_send_angle(
  auto_buff::Target & target, const double & detect_now_gap, const double bullet_speed,
  const bool to_now, double & yaw, double & pitch)
{
  // 考虑detecor所消耗的时间，此外假设aimer的用时可忽略不计
  // 如果 to_now 为 true，则根据当前时间和时间戳预测目标位置,deltatime = 现在时间减去当时照片时间，加上0.1
  target.predict(to_now ? (detect_now_gap + PREDICT_TIME) : 0.1 + PREDICT_TIME);
  angle = target.ekf_x()[5];

  // 计算目标点的空间坐标
  auto aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  double d = std::sqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
  double h = aim_in_world[2];

  // 创建弹道对象
  tools::Trajectory trajectory0(bullet_speed, d, h);
  if (trajectory0.unsolvable) {  // 如果弹道无法解算，返回未命中结果
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
    return false;
  }

  // 根据第一个弹道飞行时间预测目标位置
  target.predict(trajectory0.fly_time);
  angle = target.ekf_x()[5];

  // 计算新的目标点的空间坐标
  aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.7));
  d = fsqrt(aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
  h = aim_in_world[2];
  tools::Trajectory trajectory1(bullet_speed, d, h);
  if (trajectory1.unsolvable) {  // 如果弹道无法解算，返回未命中结果
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory1: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
    return false;
  }

  // 计算时间误差
  auto time_error = trajectory1.fly_time - trajectory0.fly_time;
  if (std::abs(time_error) > 0.01) {  // 如果时间误差过大，返回未命中结果
    tools::logger()->debug("[Aimer] Large time error: {:.3f}", time_error);
    return false;
  }

  // 计算偏航角和俯仰角，并返回命中结果
  yaw = std::atan2(aim_in_world[1], aim_in_world[0]) + yaw_offset_;
  pitch = trajectory1.pitch + pitch_offset_;
  return true;
};

void Aimer::update_status_()
{
  if (status_ == SEND_ANGLE) {
    status_ = SEND_FIRE;
  } else if (status_ == SEND_FIRE) {
    status_ = WAIT;
  } else if (status_ == WAIT) {
    status_ = SEND_ANGLE;
  }
}

void Aimer::reset_status_() { status_ = SEND_ANGLE; }

}  // namespace auto_buff