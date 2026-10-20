#pragma once

#include <Eigen/Eigenvalues>

namespace rm_task1 {

// 五个近邻必须张成二维局部表面；重复点或共线点不能确定唯一平面。
// 使用中心化散布矩阵，判定不随平移或刚体旋转改变。
template <typename Derived>
bool valid_plane_neighborhood(const Eigen::MatrixBase<Derived>& points) {
  using T = typename Derived::Scalar;
  if (!points.allFinite()) return false;
  Eigen::Matrix<T, Derived::RowsAtCompileTime, 3> centered =
      points.rowwise() - points.colwise().mean();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 3, 3>> eig(
      centered.transpose() * centered);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) return false;
  const auto& values = eig.eigenvalues();
  return values(2) >= T(1e-9) && values(1) >= T(1e-4) * values(2);
}

}  // namespace rm_task1
