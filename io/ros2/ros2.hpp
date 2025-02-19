#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#include "publish2nav.hpp"

namespace io
{
class ROS2
{
public:
  ROS2();

  ~ROS2();

  void publish(const Eigen::Vector4d & target_pos);

private:
  std::shared_ptr<Publish2Nav> publish2nav_;
  std::unique_ptr<std::thread> spin_thread_;
};

}  // namespace io
#endif