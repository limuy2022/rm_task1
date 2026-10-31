#pragma once
// 仅旋转算法输出世界系；不改原始传感器轴和雷达↔内部IMU外参。
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <vector>
namespace rm_task1 {
inline Eigen::Quaterniond world_alignment(const std::vector<double>& up) {
  if (up.size() != 3) throw std::invalid_argument("output_world_up须含三个数值");
  Eigen::Vector3d v(up[0], up[1], up[2]);
  if (!v.allFinite() || v.norm() < 1e-6)
    throw std::invalid_argument("output_world_up须为有限非零向量");
  // 最短旋转消除倾斜；不宣称恢复车头yaw，比赛yaw仍由起点及装车外参决定。
  return Eigen::Quaterniond::FromTwoVectors(v.normalized(), Eigen::Vector3d::UnitZ()).normalized();
}
}
