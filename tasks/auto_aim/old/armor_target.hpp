#ifndef AUTO_AIM__ARMOR_TARGET_HPP
#define AUTO_AIM__ARMOR_TARGET_HPP

#include <Eigen/Dense>

namespace auto_aim
{
struct ArmorTarget
{
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_TARGET_HPP